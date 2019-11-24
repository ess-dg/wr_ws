"""Class valid for interfacing XYZ cartesian CNC

ALO: See scattered TODOs. This code needs serious refactoring. GCode message parsing should be
separate function. Command termination (e.g. line feed, carriage return) should be global var.
Socket connectivity and messaging should be separate function.
"""

import time
import re
import serial
from geometry_msgs.msg import Twist


class Cnc:
    """CNC class."""
    # Regular expression for parsing GRBL status msgs
    __pos_pattern__ = re.compile(r".Pos:(\-?\d+\.\d+),(\-?\d+\.\d+),(\-?\d+\.\d+)")

    def __init__(self):
        self.serial = None  # serial port object
        self.abs_move_mode = None  # GRBL has 2 movement modes, relative and absolute
        self.baudrate = 0
        self.port = ""
        self.accel = 0
        self.x_max = 0
        self.y_max = 0
        self.z_max = 0
        self.default_speed = 0
        self.x_max_speed = 0
        self.y_max_speed = 0
        self.z_max_speed = 0
        # number of steps per centimeter in each dimension
        self.x_steps_mm = 0
        self.y_steps_mm = 0
        self.z_steps_mm = 0
        # machine idle
        self.idle = True
        # vectors follow the format [X, Y, Z] where Z is assumed to be vertical
        self.pos = [0.0, 0.0, 0.0]  # current position
        self.angular = [0.0, 0.0, 0.0]  # angular coordinates
        self.origin = [0.0, 0.0, 0.0]  # minimum coordinates
        self.limits = [0.0, 0.0, 0.0]  # maximum coordinates

    def startup(self, port, baudrate, accel, x_max, y_max, z_max, default_speed, x_max_speed,
                y_max_speed, z_max_speed, x_steps_mm, y_steps_mm, z_steps_mm):
        """Initiate CNC parameters read from .launch file."""
        self.baudrate = baudrate
        self.port = port
        self.accel = accel
        self.x_max = x_max
        self.y_max = y_max
        self.z_max = z_max
        self.default_speed = default_speed
        self.x_max_speed = x_max_speed
        self.y_max_speed = y_max_speed
        self.z_max_speed = z_max_speed
        self.x_steps_mm = x_steps_mm
        self.y_steps_mm = y_steps_mm
        self.z_steps_mm = z_steps_mm

        self.limits = [self.x_max, self.y_max, self.z_max]
        self.serial = serial.Serial(self.port, self.baudrate)
        self.ensure_move_mode(True)
        self.home()
        self.set_origin()

    def shutdown(self):
        """Close serial connection."""
        # TODO (ALO): Should probably flush buffer.
        self.serial.close()

    def get_pos(self):
        """Return a list [x,y,z] of the position of the gantry head."""
        return list(self.pos)

    def get_twist(self):
        # Convert coordinates to ROS Twist format to be able to publish it later
        cnc_pos = Twist()
        cnc_pos.linear.x = float(self.pos[0])
        cnc_pos.linear.y = float(self.pos[1])
        cnc_pos.linear.z = float(self.pos[2])
        # These parameters are set to 0 as the CNC is a XYZ 3 DOF mechanism and doesnt need them
        # ALO: ?
        cnc_pos.angular.x = float(self.angular[0])
        cnc_pos.angular.y = float(self.angular[1])
        cnc_pos.angular.z = float(self.angular[2])

        return cnc_pos

    def set_speed(self, speed):
        self.default_speed = speed

    def home(self):
        """Initiates the homing procedure."""
        self.serial.write("$H\n")
        self.serial.readline()
        self.pos = list(self.origin)

    def enable_stepper_motors(self):
        try:
            self.serial.write("M17\n")
            self.serial.readline()
        except:
            print("Serial port unavailable")

    def disable_stepper_motors(self):
        try:
            self.serial.write("M18\n")
            self.serial.readline()
        except:
            print("Serial port unavailable")

    def move_to(self, x=None, y=None, z=None, speed=None):
        """Move to an absolute position and return when movement completes."""
        if not self.idle:
            return
        if x is None and y is None and z is None:
            return

        if speed is None:
            speed = self.default_speed

        self.ensure_move_mode(abs_move_mode=True)

        gcode = 'G0'
        letters = 'XYZ'
        pos = (x, y, z)
        newpos = list(self.pos)

        #create gcode string and update position list for each argument that isn't None
        for i in range(3):
            if pos[i] is not None:
                #check against self.limits
                if pos[i] < 0 or pos[i] >= self.limits[i]:
                    # if position is outside the movement range, ignore
                    return
                gcode += ' ' + letters[i] + str(pos[i])
                newpos[i] = pos[i]

        gcode += ' F' + str(speed)
        gcode += '\n'
        try:
            self.serial.write(gcode)
            self.serial.readline()
        except:
            print("Serial port unavailable")

    def move_relative(self, dx=None, dy=None, dz=None, speed=None, block=True):
        """Move a given distance, and return when movement completes
        :param dx, dy, dz: distance to move
        :param speed: units uncertain
        :param block: whether to return immediately, or wait for the movement to complete
        """

        self.ensure_move_mode(abs_move_mode=False)
        if speed is None:
            speed = self.default_speed

        gcode = 'G0'
        letters = 'xyz'
        d = (dx, dy, dz)
        newpos = list(self.pos)

        # create gcode string and update position list for each argument that isn't None
        # TODO (pablo): if successful?
        for i in range(3):
            if d[i] is not None:
                gcode += ' ' + letters[i] + str(d[i])
                newpos[i] += d[i]

            gcode += ' f' + str(speed)
            gcode += '\n'

        self.serial.write(gcode)
        self.serial.readline()

        # the position update should be done after reading state
        # update position if success
        # TODO (pablo): check to make sure it's actually a success
        #self.pos = newpos

        if block:
            self.block_until_idle()

    def move_to_origin(self, speed=None):
        """Move to starting position, and return when movement completes."""
        if speed is None:
            speed = self.default_speed
        self.move_to(*self.origin, speed=speed)
        self.pos = list(self.origin)

    def set_origin(self, x=0, y=0, z=0):
        """Set current position to be (0,0,0), or a custom (x,y,z)."""
        gcode = "G92 x{} y{} z{}\n".format(x, y, z)
        self.serial.write(gcode)
        self.serial.readline()

        # update our internal location
        self.pos = [x, y, z]

    def ensure_move_mode(self, abs_move_mode=True):
        """GRBL has two movement modes; if necessary this function tells GRBL to switch modes."""
        if self.abs_move_mode == abs_move_mode: return

        self.abs_move_mode = abs_move_mode
        if abs_move_mode:
            self.serial.write("G90\n")  # absolute movement mode
        else:
            self.serial.write("G91\n")  # relative movement mode
            self.serial.readline()

    def block_until_idle(self):
        """Polls until GRBL indicates it is done with the last command."""
        pollcount = 0
        while True:
            self.serial.write("?")
            status = self.serial.readline()
            if status.startswith('<Idle'): break
            # not used
            pollcount += 1
            # poll every 10 ms
            time.sleep(.01)

    def get_status(self):
        self.serial.write("?")

        while True:
            try:
                status = self.serial.readline()
                if status is not None:
                    try:
                        matches = self.__pos_pattern__.findall(status)
                        if len(matches[1]) == 3:
                            self.pos = list(matches[1])
                            return status
                    except IndexError:
                        print("No matches found in serial")
                else: break
            except:
                print("Report readiness but empty")
