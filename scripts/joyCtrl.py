# control a crazyflies with joystick, record optitrack state and individual motor thrust

import logging
import time
import sys
from threading import Thread,Event,Lock
from time import time,sleep
sys.path.insert(0,'../externals/inputs/')
sys.path.insert(0,'/home/nickzhang/monocopter/optitrack_publisher')
from pylistener import Listener

import cflib
from cflib.crazyflie import Crazyflie
from inputs import get_gamepad

logging.basicConfig(level=logging.ERROR)
# TODO exit gracefully at ctrl c

class Command:
    roll = 0
    pitch = 0
    yawrate = 0
    thrust = 0


class joyCtrl:
    def __init__(self):
        # FIXME for debugging only, not the real entry point
        self.started_thread = []
        self._exit_flag = Event()
        self.init_joystick()
        self.init_optitrack_listener()

    def stay(self):
        # stays awake and wait for ctrl-C
        try:
            while (not self._exit_flag.is_set()):
                #print(self.command.roll,self.command.pitch,self.command.yawrate,self.command.thrust)
                print('.')
                print(self.optitrack_state.x)
                sleep(0.02)
        except (KeyboardInterrupt,SystemExit):
            print("exit")
        finally:
            self._exit_flag.set()
            for thread in self.started_thread:
                thread.join()
            exit(0)

    def __realinit__(self, link_uri):
        """ Initialize and run the script with the specified link_uri """

        self._cf = Crazyflie(rw_cache='./cache')

        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        self._cf.add_port_callback(CRTPPort.MOTOR, self._incoming)
        self._cf.open_link(link_uri)
        self._cf.commander.send_setpoint(0, 0, 0, 0)

        self.started_thread = []
        self._exit_flag = Event()
        self.init_joystick()
        self.init_optitrack_listener()

        # TODO listen to optitrack in a separate thread

        print('Connecting to %s' % link_uri)

    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""

        # Start a separate thread to do the motor test.
        # Do not hijack the calling thread!
        # Thread(target=self._ramp_motors).start()

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the specified address)"""
        self._exit()
        print('Connection to %s failed: %s' % (link_uri, msg))

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        self._exit()
        print('Disconnected from %s' % link_uri)

    def _exit(self):
        # exit gracefully
        self._cf.commander.send_setpoint(0, 0, 0, 0)
        # Make sure that the last packet leaves before the link is closed
        # since the message queue is not flushed before closing
        time.sleep(0.1)
        self._exit_flag.set()
        # TODO join all threads
        self._cf.close_link()
        return

    def init_optitrack_listener(self):
        # TODO initialize a thread to update optitrack
        # TODO register this thread in exit
        self.optitrack_state = None
        self.optitrack_listener_thread = Thread(name="optitrack",target=self._optitrack_listener)
        self.optitrack_listener_thread.start()
        self.started_thread.append(self.optitrack_listener_thread)
        pass

    def _optitrack_listener(self):
        # TODO handle exception
        # .x,.y,.z,.roll,.pitch,.yaw
        while (not self._exit_flag.is_set()):
            self.optitrack_state = listener.ReceivePackage()
            #print(self.optitrack_state.x)
            self.writeLog()

    def writeLog(self):
        # TODO maintain log
        pass


    def _incoming(self,packet):
        # TODO process incoming CRTP packet, (PORT=MOTOR), update local state
        pass

    def calibrate_joystick(self):
        # TODO add calibration
        pass

    def init_joystick(self):
        # TODO add error handling
        self.gamepad_lock = Lock()
        self.gamepad = {}
        self.gamepad['ABS_Y'] = 0
        self.gamepad['ABS_X'] = 0
        self.gamepad['ABS_RX'] = 0
        self.gamepad['ABS_RY'] = 0

        self.command_lock = Lock()
        self.command = Command()
        self.joystick_update_thread = Thread(name="joystickUpdate",target=self._update_joystick)
        self.joystick_update_thread.start()
        self.started_thread.append(self.joystick_update_thread)
        print('Joystick init success')
        return

    def joymap(self,val):
        # map joystick value from (-32767,32767) to (-1.0,1.0)
        retval = float(val)/32767.0
        retval = min(retval,1.0)
        retval = max(retval,-1.0)
        return retval

    def _update_joystick(self):
        while (not self._exit_flag.is_set()):
            # TODO map input and send packet for setpoint
            # TODO handle exception
            events = get_gamepad()
            for event in events:
                if (event.ev_type == 'Absolute'):
                    self.gamepad_lock.acquire()
                    self.gamepad[event.code] = event.state
                    self.gamepad_lock.release()
                    #print(event.code,event.state)
            # map joystick value to roll/pitch/yaw/thrust
            self.command.roll = self.joymap(self.gamepad['ABS_RX'])
            self.command.pitch = self.joymap(self.gamepad['ABS_RY'])
            self.command.yawrate = self.joymap(self.gamepad['ABS_X'])
            self.command.thrust = self.joymap(self.gamepad['ABS_Y'])
        return

    def _main(self):
        # Unlock startup thrust protection
        self._cf.commander.send_setpoint(0, 0, 0, 0)
        sleep(0.1)

        # periodically send setpoints to crazyflies
        while (not self._exit_flag.is_set()):
            self.command_lock.acquire()
            self._cf.commander.send_setpoint(self.command.roll,self.command.pitch,self.command.yawrate,self.command.thrust)
            self.command_lock.release()
            # keep update frequency at 100Hz, as recommended by crazyfiles
            sleep(0.01)

# main entry point
if __name__ == '__realmain__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)
    # Scan for Crazyflies and use the first one found
    print('Scanning interfaces for Crazyflies...')
    available = cflib.crtp.scan_interfaces()
    print('Crazyflies found:')
    for i in available:
        print(i[0])

    if len(available) > 0:
        le = joyCtrl(available[0][0])
    else:
        print('No Crazyflies found, cannot run example')


# FIXME For Debug
if __name__ == '__main__':
    jc = joyCtrl()
    jc.stay()
    





