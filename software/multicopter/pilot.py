"""
To install X-Box gamepad software on Ubuntu >=14.04:
    sudo apt-get install dkms
    sudo git clone https://github.com/paroj/xpad.git /usr/src/xpad-0.4
    sudo dkms install -m xpad -v 0.4
    sudo modprobe xpad (might need to turn off "secure boot" in BIOS)
    reboot computer and then plug in an Xbox 360/One gamepad via USB
    press the big X button, light should glow solid (not flashing)

"""
from __future__ import division
import numpy as np; npl = np.linalg

from threading import Thread
from collections import deque

from inputs import devices, get_gamepad
from allocator import Allocator
import motion


class Pilot(object):
    """
    User interface for remote-controlling a multicopter.
    Call start_pilot_thread to begin filling an internal buffer with user input.
    Call control to get the efforts dictionary that should be implemented by the multicopter.
    Don't forget to call stop_pilot_thread when done!

    model:            a Model object for the multicopter
    kp:               list of proportional gains for these degrees of freedom [world_up, roll, pitch, yaw]
    kd:               list of derivative gains for these degrees of freedom [world_up, roll, pitch, yaw]
    max_roll:         magnitude of the largest acceptable roll command (in degrees)
    max_pitch:        magnitude of the largest acceptable pitch command (in degrees)
    max_yaw_rate:     magnitude of the largest acceptable yaw rate command (in degrees per time)
    max_ascent_rate:  magnitude of the largest acceptable ascent rate command
    stick_deadband:   fraction of analog joystick travel that should be treated as zero
    trigger_deadband: fraction of analog trigger travel that should be treated as zero
    max_buffer_size:  maximum number of user commands that should be stored before dropping old ones
    button_callbacks: dictionary of callback functions keyed by button names (A, B, X, Y, L, R, SL, SR, DV, DH)

    """
    def __init__(self, model, kp, kd, max_roll=65, max_pitch=65, max_yaw_rate=180, max_ascent_rate=5,
                 stick_deadband=0.1, trigger_deadband=0.0, max_buffer_size=200, button_callbacks={}):
        self.model = model
        self.kp = np.array(kp, dtype=np.float64)
        self.kd = np.array(kd, dtype=np.float64)
        self.max_roll = np.deg2rad(max_roll)
        self.max_pitch = np.deg2rad(max_pitch)
        self.max_yaw_rate = np.deg2rad(max_yaw_rate)
        self.max_ascent_rate = np.float64(max_ascent_rate)
        self.stick_deadband = float(stick_deadband)
        self.trigger_deadband = float(trigger_deadband)
        self.max_buffer_size = int(max_buffer_size)
        self.button_callbacks = button_callbacks

        # Valid input device names in priority order
        self.valid_device_names = ["Microsoft X-Box One pad (Firmware 2015)"]

        # Set valid input device
        self.input_device = None
        for valid_device_name in self.valid_device_names:
            if self.input_device is not None: break
            for device in devices:
                if device.name == valid_device_name:
                    self.input_device = device.name
                    print "Hello, Pilot! Ready to read from {}.".format(device.name)
                    break
        if self.input_device is None: raise IOError("FATAL: No valid input device is connected!")

        # Digital button code names
        self.button_codes = {"A": "BTN_SOUTH", "B": "BTN_EAST", "X": "BTN_NORTH", "Y": "BTN_WEST",
                             "L": "BTN_TL", "R": "BTN_TR", "SL": "BTN_SELECT", "SR": "BTN_START",
                             "DV": "ABS_HAT0Y", "DH": "ABS_HAT0X"}

        # Analog input characteristics
        self.max_stick = 32767
        self.max_trigger = 1023
        self.min_stick = int(self.stick_deadband * self.max_stick)
        self.min_trigger = int(self.trigger_deadband * self.max_trigger)

        # For use by control
        self.allocator = Allocator(self.model, verbose=False)

        # Internals
        self.cmd = None
        self.pilot_thread = None
        self.stay_alive = False
        self.buffer = deque([])
        self.buffer_size_flag = False

    def control(self, state):
        """
        Executes the internal input buffer to bring the current pilot commands up to date.
        Returns the efforts that should be applied to achieve the current pilot commands.

        state: State object with the multicopter's current state

        """
        # Execute (and clear) command buffer to update internals
        if self.pilot_thread is None: raise AssertionError("FATAL: Cannot do pilot control until you start_pilot_thread!")
        self._execute_buffer(state.time - self.cmd.time)

        # Project body-up direction onto world-up direction (get cosine of the tilt angle)
        ctilt = self.cmd.ascent_dir.dot(state.pose.rotate_vector([0, 0, 1]))

        # Don't worry about ascent if completely sideways or upsidedown
        if ctilt <= 1e-5:
            response_force = [0, 0, 0]
        else:
            # Compute actual state's world-coordinate ascent rate
            state_ascent_rate = self.cmd.ascent_dir.dot(state.pose.rotate_vector(state.twist.lin))
            # Let thruster force be completely along body-up and strong enough to cancel world-coordinate gravity, plus feedback
            response_force = [0, 0, (1/ctilt) * (self.model.mass*self.model.gmag +
                                                 self.kp[0]*(self.cmd.ascent - self.cmd.ascent_dir.dot(state.pose.lin)) +
                                                 self.kd[0]*(self.cmd.ascent_rate - state_ascent_rate))]

        # Ordinary Lie algebraic attitude control scheme for thruster torques
        qdes = motion.quaternion_from_euler(self.cmd.roll, self.cmd.pitch, self.cmd.yaw)
        response_torque = self.kp[1:]*motion.ori_error(qdes, state.pose.ang) + self.kd[1:]*([0, 0, self.cmd.yaw_rate] - state.twist.ang)

        # Allocate efforts for the response wrench
        return self.allocator.allocate(motion.Wrench(response_force, response_torque))

    def start_pilot_thread(self, primer_state):
        """
        Starts a thread that reads user input and stores the commands into the internal buffer.

        primer_state: State object that determines the initial yaw and ascent commands, and time

        """
        if self.stay_alive:
            print "----------"
            print "WARNING: Pilot thread already running!"
            print "Cannot start another."
            print "----------"
            return
        self.cmd = CmdState(primer_state, ascent_dir=-self.model.gdir)
        self.stay_alive = True
        if self.input_device == "Microsoft X-Box One pad (Firmware 2015)":
            self.pilot_thread = Thread(target=self._listen_xbox)
        else:
            raise IOError("FATAL: No listener function has been implemented for device {}.".format(self.input_device))
        print "Pilot thread has begun!"
        self.pilot_thread.start()

    def stop_pilot_thread(self):
        """
        Terminates the Pilot's user input reading thread and clears the buffer.

        """
        self.stay_alive = False
        if self.pilot_thread is not None:
            print "Pilot thread terminating on next input!"
            self.pilot_thread.join()  # stay secure
            print "Pilot thread terminated!"
            self.pilot_thread = None
        while self.buffer:
            self.buffer.pop()
        self.buffer_size_flag = False
        self.cmd = None

    def _listen_xbox(self):
        try:
            while self.stay_alive:
                self.buffer.appendleft(get_gamepad()[0])  # this is blocking (hence need for threading)
                if len(self.buffer) > self.max_buffer_size:
                    if not self.buffer_size_flag:
                        self.buffer_size_flag = True
                        print "----------"
                        print "WARNING: Pilot input buffer reached {} entries.".format(self.max_buffer_size)
                        print "Dropping old commands."
                        print "----------"
                    self.buffer.pop()
        except IOError:
            raise IOError("PROBABLY FATAL: User input device disconnected.\nTerminating Pilot thread.")  # better error message

    def _execute_buffer(self, dt):
        while self.buffer:
            event = self.buffer.pop()

            # Analog inputs
            if event.code == "ABS_Y": self.cmd.ascent_rate = -self._stick_frac(event.state) * self.max_ascent_rate
            elif event.code == "ABS_X": pass
            elif event.code == "ABS_RY": self.cmd.pitch = -self._stick_frac(event.state) * self.max_pitch
            elif event.code == "ABS_RX": self.cmd.roll = self._stick_frac(event.state) * self.max_roll
            elif event.code == "ABS_Z": self.cmd.yaw_rate = self._trigger_frac(event.state) * self.max_yaw_rate
            elif event.code == "ABS_RZ": self.cmd.yaw_rate = -self._trigger_frac(event.state) * self.max_yaw_rate
            else:
                # Digital inputs
                for btn in self.button_callbacks:
                    if self.button_codes[btn] == event.code:
                        self.button_callbacks[btn](event.state)

        # Update integrated command states
        self.cmd.time = self.cmd.time + dt
        self.cmd.yaw = motion.unwrap_angle(self.cmd.yaw + dt*self.cmd.yaw_rate)
        self.cmd.ascent = self.cmd.ascent + dt*self.cmd.ascent_rate

    def _stick_frac(self, val):
        if abs(val) > self.min_stick:
            return np.divide(val, self.max_stick, dtype=np.float64)
        return np.float64(0)

    def _trigger_frac(self, val):
        if abs(val) > self.min_trigger:
            return np.divide(val, self.max_trigger, dtype=np.float64)
        return np.float64(0)


class CmdState(object):
    """
    Structure for neatly holding the state of the command inside a Pilot.

    primer_state: State object that determines the initial yaw and ascent commands, and time
    ascent_dir:   direction vector defining up in world-coordinates (opposite of gravity direction)

    """
    def __init__(self, primer_state, ascent_dir=[0, 0, 1]):
        ascent_dir_mag = npl.norm(ascent_dir)
        if ascent_dir_mag == 0:
            print "----------"
            print "WARNING: Pilot's ascent direction is poorly defined."
            print "Using world-coordinate z direction."
            print "----------"
            self.ascent_dir = np.array([0, 0, 1], dtype=np.float64)
        else:
            self.ascent_dir = np.array(ascent_dir, dtype=np.float64) / ascent_dir_mag
        self.reset(primer_state)

    def reset(self, primer_state):
        self.time = primer_state.time
        self.roll = np.float64(0)
        self.pitch = np.float64(0)
        _, _, self.yaw = motion.euler_from_quaternion(primer_state.pose.ang)
        self.yaw_rate = np.float64(0)
        self.ascent = self.ascent_dir.dot(primer_state.pose.lin)
        self.ascent_rate = np.float64(0)
