"""
Dependency installation for Ubuntu >=14.04:
    http://twistedmatrix.com/trac/wiki/Downloads
    https://www.digi.com/resources/documentation/Digidocs/90001110-88/tasks/t_download_and_install_xctu_linux.htm

"""
from __future__ import division
import struct
from twisted.internet import reactor, defer
from deferral import wrap_timeout, TimeoutError, sleep
from xbee import XBee
from multicopter.motion import Command


class Radio(object):
    """
    Handles both transmitting and receiving data over an XTend-PKG RF modem.
    Call transmit_command to broadcast a Command object.
    Call start_receiver_coroutine to listen for broadcasted Command objects and pass them to a given callback.
    Call stop_receiver_coroutine to cease listening (terminate coroutine).
    While the receiver coroutine is running, the main program from which start_receiver_coroutine
    was called needs to either be a loop that frequently calls reactor.iterate or a script
    that is kept alive with a call to reactor.run at the end.

    port:        string with USB port path name
    baud:        data baud rate
    rcv_timeout: time in seconds between receipts before connection is assumed to be lost

    """
    def __init__(self, port="/dev/ttyUSB0", baud=230400, rcv_timeout=0.1):
        self.port = str(port)
        self.baud = int(baud)
        self.rcv_timeout = float(rcv_timeout)

        # Create XBee object (for simplicity, this is blocking)
        self.xb = None
        @defer.inlineCallbacks
        def eventually_create_xb():
            print "Initializing XBee..."
            print "(If any 'OK\\r' are missing, check device)."
            print "----"
            self.xb = yield XBee(reactor, self.port, self.baud)
            print "----"
            print "XBee initialized! (assuming all 'OK\\r' are there)"
        eventually_create_xb()
        while self.xb is None:
            reactor.iterate()

        # Internals
        self.packet_config = "<4fHBB"
        self.command = Command()
        self.receiver_callback = None
        self.stay_alive = False
        self.connection = False

    def transmit_command(self, command):
        """
        Transmits command object data serially from the radio.

        command: Command object

        """
        self.xb.transmit(struct.pack(self.packet_config, command.roll, command.pitch, command.yaw_rate, command.ascent_rate,
                                     command.start, command.cancel, command.kill))

    def start_receiver_coroutine(self, callback):
        """
        Begins a coroutine that listens for radio data, packs it into a Command object, and passes it to a callback.

        callback: function that takes a Command object and does whatever with it

        """
        if self.stay_alive:
            print "----------"
            print "WARNING: This Radio's receiver coroutine is already running!"
            print "Cannot start another."
            print "----------"
            return
        self.stay_alive = True
        self.set_receiver_callback(callback)
        print "Receiver coroutine has begun!"
        self._receiver_coroutine()

    def stop_receiver_coroutine(self):
        """
        Terminates the receiver coroutine.

        """
        self.stay_alive = False

    def set_receiver_callback(self, callback):
        """
        Changes the receiver callback to the function provided.

        callback: function that takes a Command object and does whatever with it

        """
        self.receiver_callback = callback

    @defer.inlineCallbacks
    def _receiver_coroutine(self):
        while self.stay_alive:
            try:
                packet, = yield wrap_timeout(self.xb.packet_received.get_deferred(), self.rcv_timeout)  # the comma IS important
            except TimeoutError:
                self.command = Command(cancel=self.connection)
                if self.connection:
                    print "Connection lost!"
                    self.connection = False
            else:
                if not self.connection:
                    print "Connection found!"
                    self.connection = True
                data = struct.unpack(self.packet_config, packet["data"])
                self.command = Command(data[0], data[1], data[2], data[3], data[4], data[5], data[6])
                self.receiver_callback(self.command)
        self.connection = False
        print "Receiver coroutine terminated!"
