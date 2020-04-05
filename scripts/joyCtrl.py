# control a crazyflies with joystick, record optitrack state and individual motor thrust
# note, if optitrack is not available, the listner function may block indefinitely and refuse to exit, in such case, use kill or restart optitrack feed

import logging
import sys
import os
from threading import Thread,Event,Lock
from time import time,sleep
sys.path.insert(0,'../externals/inputs/')
# TODO remove this
sys.path.insert(0,'/home/nickzhang/monocopter/optitrack_publisher')
from pylistener import Listener

import cflib
from cflib.crazyflie import Crazyflie
from inputs import get_gamepad,UnpluggedError

logging.basicConfig(level=logging.ERROR)

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
        self.log_is_ready = Event()
        self.init_joystick()
        self.init_optitrack_listener()
        self.reported_motor_thrust = (0,0,0,0)
        self.reported_voltage = 0.0
        self.init_log()

    def stay(self):
        # stays awake and wait for ctrl-C
        try:
            while (not self._exit_flag.is_set()):
                #print(self.command.roll,self.command.pitch,self.command.yawrate,self.command.thrust)
                #print('.')
                #print(self.optitrack_state.x)
                sleep(0.02)
        except (KeyboardInterrupt,SystemExit):
            print("exiting... Please press Ctrl-C multiple times")
        finally:
            self.write_log(sync=True)
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

        # register our callback function
        self._cf.add_port_callback(CRTPPort.MOTOR, self._incoming)
        self._cf.open_link(link_uri)
        self._cf.commander.send_setpoint(0, 0, 0, 0)

        self.started_thread = []
        self._exit_flag = Event()
        self.init_joystick()
        self.init_optitrack_listener()

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
        sleep(0.1)
        self._exit_flag.set()
        self._cf.close_link()

        self.write_log(sync=True)
        for thread in self.started_thread:
            thread.join()
        exit(0)

    def init_optitrack_listener(self):
        # initialize a thread to update optitrack
        self.optitrack_state = None
        self.optitrack_listener = Listener()
        self.optitrack_listener_thread = Thread(name="optitrack",target=self._optitrack_listener)
        self.optitrack_listener_thread.start()
        self.started_thread.append(self.optitrack_listener_thread)
        return

    def _optitrack_listener(self):
        # handle exception
        # .x,.y,.z,.roll,.pitch,.yaw
        while (not self._exit_flag.is_set()):
            try:
                self.optitrack_state = self.optitrack_listener.ReceivePackage()
                #print(self.optitrack_state.x)
                self.write_log()
            except (KeyboardInterrupt,SystemExit):
                self._exit_flag.set()
                exit(0)

    def init_log(self):
        logFolder = "./log/"
        logPrefix = "richrun"
        logSuffix = ".txt"
        if (not os.path.isdir(logFolder)):
            print("log folder does not exist, creating one")
            os.makedirs(logFolder)

        no = 1
        self.logBuffer = []
        while os.path.isfile(logFolder+logPrefix+str(no)+logSuffix):
            no += 1
        self.logFilename = logFolder+logPrefix+str(no)+logSuffix
        print("log writing to: "+self.logFilename)
        sleep(1)
        self.log_is_ready.set()



    def write_log(self,sync=False):
        # maintain log
        # TODO Lock?
        if not self.log_is_ready.is_set():
            return
        logEntry = str(time())+","+str(self.optitrack_state.x)+","+str(self.optitrack_state.y)+","+str(self.optitrack_state.z)+","+str(self.optitrack_state.roll)+","+str(self.optitrack_state.pitch)+","+str(self.optitrack_state.yaw)+","+str(self.reported_motor_thrust[0])+","+str(self.reported_motor_thrust[1])+","+str(self.reported_motor_thrust[2])+","+str(self.reported_motor_thrust[3])+str(self.reported_voltage)+"\n"
        self.logBuffer.append(logEntry)
        print(logEntry)
        if (sync or len(self.logBuffer)>300):
            with open(self.logFilename, 'a') as filehandle:
                for entry in self.logBuffer:
                    filehandle.write('%s' % entry)
            self.logBuffer = []
        return


    def _incoming(self,packet):
        # TODO process incoming CRTP packet, (PORT=MOTOR), update local state
        # TODO verify this is correct
        # channel 1: thrust command to execute, pc->crazyflie, for use with control
        # channel 2: thrust report, crazyflie->pc, for logging
        # package data field formatting:
        # Byte 0-1 : thrust 0, uint16_t, this is the value sent to motorsSetRatio (ithrust), 65536->60grams
        # 2-3: thrust 1
        # 4-5: 2
        # 6-7: 3
        # Byte 8-11 : supply voltage, float, from pmGetBatteryVoltage()
        # the actual equation to convert ithrust to PWM can be found in crazyflie-firmware/src/drivers/src/motors.c
        if (packet.channel==2):
            # NOTE < little endian, H: unsigned short 2Bytes, B: unsigned char 1 Byte, f: float 4 byte, all used in crtp protocol
            self.reported_motor_thrust = struct.unpack('<HHHH', packet.data[:8])
            self.reported_voltage =  struct.unpack('<f', packet.data[8:])

    def calibrate_joystick(self):
        # TODO add calibration
        pass

    def init_joystick(self):
        self.gamepad_lock = Lock()
        self.gamepad = {}
        self.gamepad['ABS_Y'] = 0
        self.gamepad['ABS_X'] = 0
        self.gamepad['ABS_RX'] = 0
        self.gamepad['ABS_RY'] = 0

        self.command_lock = Lock()
        self.command = Command()

        print("please press any button on gamepad to complete init")
        try:
            get_gamepad()
        except UnpluggedError as e:
            print("Gamepad Init Error: "+str(e))
            self._exit_flag.set()
            return

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
            try:
                try:
                    events = get_gamepad()
                except UnpluggedError as e:
                    error(e)
                    self._exit_flag.set()
                    return
                for event in events:
                    if (event.ev_type == 'Absolute'):
                        self.gamepad_lock.acquire()
                        self.gamepad[event.code] = event.state
                        self.gamepad_lock.release()
                        #print(event.code,event.state)
                # map joystick value to roll/pitch/yaw/thrust
                # TODO all values are in range (-1.0,1.0), map them to appropriate values
                self.command_lock.acquire()
                self.command.roll = self.joymap(self.gamepad['ABS_RX'])
                self.command.pitch = self.joymap(self.gamepad['ABS_RY'])
                self.command.yawrate = self.joymap(self.gamepad['ABS_X'])
                self.command.thrust = self.joymap(self.gamepad['ABS_Y'])
                self.command_lock.release()
                # NOTE _main() will send command to crazyfly
            except (KeyboardInterrupt,SystemExit):
                self._exit_flag.is_set()
                exit(0)
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
    





