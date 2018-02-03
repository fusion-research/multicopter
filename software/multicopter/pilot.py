"""
To install X-Box gamepad software on Ubuntu 14.04:
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
from collections import deque  # thread safe
from inputs import devices, get_gamepad
from motion import Command


class Pilot(object):
    """
    User interface for remote-controlling a multicopter.
    Call start_pilot_thread to begin filling an internal buffer with user input.
    Call get_command to execute / clear the buffer and get the current relevant Command object.
    Change the mission_code attribute to an integer that will be sent as command.start on activation.
    Call stop_pilot_thread when done!

    max_roll:         magnitude of the largest acceptable roll command (in degrees)
    max_pitch:        magnitude of the largest acceptable pitch command (in degrees)
    max_yaw_rate:     magnitude of the largest acceptable yaw rate command (in degrees per time)
    max_ascent_rate:  magnitude of the largest acceptable ascent rate command
    stick_deadband:   fraction of analog joystick travel that should be treated as zero
    trigger_deadband: fraction of analog trigger travel that should be treated as zero
    max_buffer_size:  maximum number of user commands that should be stored before dropping old ones
    button_callbacks: dictionary of callback functions keyed by button names (A, B, X, Y, L, R, SL, SR, DV, DH, K)

    """
    def __init__(self, max_roll=65, max_pitch=65, max_yaw_rate=180, max_ascent_rate=5,
                 stick_deadband=0.1, trigger_deadband=0.0, max_buffer_size=200, button_callbacks={}):
        self.max_roll = np.deg2rad(max_roll)
        self.max_pitch = np.deg2rad(max_pitch)
        self.max_yaw_rate = np.deg2rad(max_yaw_rate)
        self.max_ascent_rate = np.float64(max_ascent_rate)
        self.stick_deadband = float(stick_deadband)
        self.trigger_deadband = float(trigger_deadband)
        self.max_buffer_size = int(max_buffer_size)
        self.button_callbacks = button_callbacks

        # Valid input device names in priority order
        self.valid_device_names = ["Microsoft X-Box One pad (Firmware 2015)",
                                   "PowerA Xbox One wired controller"]

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
        self.button_codes = {"BTN_SOUTH": "A", "BTN_EAST": "B", "BTN_NORTH": "X", "BTN_WEST": "Y",
                             "BTN_TL": "L", "BTN_TR": "R", "BTN_SELECT": "SL", "BTN_START": "SR",
                             "ABS_HAT0Y": "DV", "ABS_HAT0X": "DH", "BTN_MODE": "K"}

        # Analog input characteristics
        self.max_stick = 32767
        self.max_trigger = 1023
        self.min_stick = int(self.stick_deadband * self.max_stick)
        self.min_trigger = int(self.trigger_deadband * self.max_trigger)

        # Internals
        self.command = None
        self.pilot_thread = None
        self.stay_alive = False
        self.buffer = deque([])
        self.buffer_size_flag = False

        # Change this integer attribute to affect what command.start will be when activated
        self.mission_code = 0

    def get_command(self):
        """
        Executes / clears the input buffer and returns the current relevant Command object.

        """
        if self.pilot_thread is None: raise AssertionError("FATAL: Cannot get_command without active pilot thread!")
        while self.buffer:
            event = self.buffer.pop()
            if event.code == "ABS_Y": self.command.ascent_rate = -self._stick_frac(event.state) * self.max_ascent_rate
            elif event.code == "ABS_X": pass
            elif event.code == "ABS_RY": self.command.pitch = -self._stick_frac(event.state) * self.max_pitch
            elif event.code == "ABS_RX": self.command.roll = self._stick_frac(event.state) * self.max_roll
            elif event.code == "ABS_Z": self.command.yaw_rate = self._trigger_frac(event.state) * self.max_yaw_rate
            elif event.code == "ABS_RZ": self.command.yaw_rate = -self._trigger_frac(event.state) * self.max_yaw_rate
            elif event.code in self.button_codes:
                if event.code == "BTN_WEST": self.command.start = int(event.state * self.mission_code)
                elif event.code == "BTN_NORTH": self.command.cancel = bool(event.state)
                elif event.code == "BTN_MODE": self.command.kill = bool(event.state)
                self.button_callbacks.get(self.button_codes[event.code], lambda val: None)(event.state)
        return self.command

    def start_pilot_thread(self):
        """
        Starts a thread that reads user input into the internal buffer.

        """
        if self.stay_alive:
            print "----------"
            print "WARNING: Pilot thread already running!"
            print "Cannot start another."
            print "----------"
            return
        self.command = Command()
        self.stay_alive = True
        if self.input_device in ["Microsoft X-Box One pad (Firmware 2015)",
                                 "PowerA Xbox One wired controller"]:
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
            self.pilot_thread = None
        while self.buffer:
            self.buffer.pop()
        self.buffer_size_flag = False
        self.command = None

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
        finally:
            print "Pilot thread terminated!"
            self.pilot_thread = None

    def _stick_frac(self, val):
        if abs(val) > self.min_stick:
            return np.divide(val, self.max_stick, dtype=np.float64)
        return np.float64(0)

    def _trigger_frac(self, val):
        if abs(val) > self.min_trigger:
            return np.divide(val, self.max_trigger, dtype=np.float64)
        return np.float64(0)
