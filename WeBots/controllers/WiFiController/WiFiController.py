from controller import Robot
import json
import sys
import socket
import numpy as np

# Initial robot position
START_NODE = 'N'
START_DIRECTION = 0.0 # 0.0 = right, np.pi = left, 0.5*np.pi = up, -0.5*np.pi = down 

# Goal node
GOAL_NODE = 'Q'

# WiFi Connection settings
ESP32_IP = '192.168.178.247'
ESP32_PORT = 8888

# Motion settings
BASE_SPEED = 0.3 * 6.28

# PID controller parameters
Kp = 0.75
Ki = 0.001
Kd = 0.08

# States
STATE_FOLLOW = 0
STATE_CENTER_JUNCTION = 1
STATE_TURN = 2
STATE_FINISHED = 3

# e-puck Physical parameters for the kinematics model (constants)
R = 0.020    # radius of the wheels: 20.5mm [m]
D = 0.057    # distance between the wheels: 52mm [m]
OFFSET_SENSOR_ROBOTCENTER = 0.038

# Set map locations
locations = {
    'A': (0.185, -0.35),
    'B': (0.293, -0.35), 
    'C': (0.395, -0.35),
    'D': (0.5, -0.35),
    'E': (0.185, -0.25),
    'F': (0.293, -0.25),
    'G': (0.395, -0.25),
    'H': (0.5, -0.25),
    'I': (-0.5, -0.25),
    'J': (0.0, -0.25),
    'K': (-0.5, -0.1),
    'L': (0.0, -0.1),
    'M': (-0.5, 0.0),
    'N': (0.0, 0.0),
    'O': (0.5, 0.0),
    'P': (0.0, 0.1),
    'Q': (0.5, 0.1),
    'R': (0.5, 0.25),
    'S': (0.0, 0.25),
    'T': (-0.185, 0.25),
    'U': (-0.293, 0.25),
    'V': (-0.395, 0.25),
    'W': (-0.5, 0.25),
    'X': (-0.5, 0.35),
    'Y': (-0.395, 0.35),
    'Z': (-0.293, 0.35),
    'AA': (-0.185, 0.35)
}

def normalize_angle(angle):
    """
    Normalizes an angle to the range [-pi, pi)
    
    Args:
        angle (float): The angle in radians to normalize.
    
    Returns:
        float: The normalized angle in radians.
    """
    if angle >= np.pi:
        angle = angle - 2*np.pi
    elif angle < -np.pi:
        angle = angle + 2*np.pi
        
    return angle

class Position:
    """
    Class representing a position in the environment.
    
    Attributes:
        x (float): The x-coordinate of the position.
        y (float): The y-coordinate of the position.
        phi (float): The orientation angle in radians.
    """	
    def __init__(self, x, y, phi):
        """
        Initializes a Position instance.
        
        Args:
            x (float): The x-coordinate of the position.
            y (float): The y-coordinate of the position.
            phi (float): The orientation angle in radians.
        """
        self.x = x
        self.y = y
        self.phi = phi
        
    @classmethod
    def from_node(cls, node):
        """
        Creates a Position instance from a node identifier.
        
        Args:
            node (str): The node identifier.
        
        Returns:
            Position: A new Position instance with coordinates from the locations dictionary.
        """
        x, y = locations[node]
        return cls(x, y, 0)
    
    @classmethod
    def from_pos(cls, pos):
        """
        Creates a Position instance from another Position instance.

        Args:
            pos (Position): The Position instance to copy.

        Returns:
            Position: A new Position instance with the same coordinates and orientation.
        """
        return cls(pos.x, pos.y, pos.phi)
    
    def __str__(self):
        """
        Returns a string representation of the Position instance.
        
        Returns:
            str: A string representation of the position in the format "Position: x:{x}, y:{y}, phi:{phi}".
        """
        return f"Position: x:{self.x}, y:{self.y}, phi:{self.phi}"
    
class PuckRobot(Position):
    """
    Class representing a puck robot in the environment.
    Inherits from Position and includes methods for sensor readings and movement.

    Attributes:
        robot (Robot): The Webots robot instance.
        timestep (int): The time step for the robot's control loop.
        linesensor_left (Device): The left line sensor.
        linesensor_center (Device): The center line sensor.
        linesensor_right (Device): The right line sensor.
        encoder_left (Device): The left wheel encoder.
        encoder_right (Device): The right wheel encoder.
        leftMotor (Device): The left wheel motor.
        rightMotor (Device): The right wheel motor.
        delta_t (float): Time step in seconds for calculations.
    """
    def __init__(self, robot, timestep, start_node, start_direction):
        """
        Initializes a PuckRobot instance.
        
        Args:
            robot (Robot): The Webots robot instance.
            timestep (int): The time step for the robot's control loop.
            start_node (str): The starting node identifier.
            start_direction (float): The starting direction in radians.
        """
        x, y = locations[start_node]
        super().__init__(x, y, start_direction)
        self.robot = robot
        self.delta_t = timestep/1000.0
        
        # Initialize linesensors
        self.linesensor_left = self.robot.getDevice('gs0')
        self.linesensor_left.enable(timestep)
        self.linesensor_center = self.robot.getDevice('gs1')
        self.linesensor_center.enable(timestep)
        self.linesensor_right = self.robot.getDevice('gs2')
        self.linesensor_right.enable(timestep)

        # Initialize encoders
        self.encoder_left = self.robot.getDevice('left wheel sensor')
        self.encoder_left.enable(timestep)
        self.encoder_right = self.robot.getDevice('right wheel sensor')
        self.encoder_right.enable(timestep)

        # Initialize encoder positions, used to track difference in encoder values over time
        self.encoder_left_pos = 0
        self.encoder_right_pos = 0

        # Initialize motors
        self.leftMotor = self.robot.getDevice('left wheel motor')
        self.rightMotor = self.robot.getDevice('right wheel motor')
        self.leftMotor.setPosition(float('inf'))
        self.rightMotor.setPosition(float('inf'))
        self.leftMotor.setVelocity(0.0)
        self.rightMotor.setVelocity(0.0)

    def line_left(self):
        """Checks if the left line sensor detects a line."""
        return self.linesensor_left.getValue() < 600

    def line_center(self):
        """Checks if the center line sensor detects a line."""
        return self.linesensor_center.getValue() < 600

    def line_right(self):
        """Checks if the right line sensor detects a line."""
        return self.linesensor_right.getValue() < 600

    def update_robot_pose(self):
        """Updates robot pose based on heading and linear and angular speeds"""
        # Get new encoder values
        new_encoder_left = self.encoder_left.getValue()
        new_encoder_right = self.encoder_right.getValue()
            
        # Get wheel speeds [Rad/s]
        wl = (new_encoder_left - self.encoder_left_pos)/self.delta_t
        wr = (new_encoder_right - self.encoder_right_pos)/self.delta_t
        
        # Update encoder positions
        self.encoder_left_pos = new_encoder_left
        self.encoder_right_pos = new_encoder_right
        
        # Get robot speed 
        u = R/2.0 * (wr + wl)
        w = R/D * (wr - wl)
        
        # Calculate changes in orientation and position, update robot pose
        delta_phi = w * self.delta_t
        self.phi = normalize_angle(self.phi + delta_phi)    
        
        delta_x = u * np.cos(self.phi) * self.delta_t
        delta_y = u * np.sin(self.phi) * self.delta_t    
        self.x += delta_x
        self.y += delta_y
    
    def update_wheel_speeds(self, left_speed, right_speed):
        """Updates the wheel speeds of the robot"""
        self.leftMotor.setVelocity(left_speed)
        self.rightMotor.setVelocity(right_speed)
    

class PIDController:
    """
    Class representing a PID controller for controlling the robot's movement.
    
    Attributes:
        Kp (float): Proportional gain.
        Ki (float): Integral gain.
        Kd (float): Derivative gain.
        integral (float): Integral of the error.
        last_error (float): Last error value for derivative calculation.
    """

    def __init__(self, Kp, Ki, Kd):
        """
        Initializes a PIDController instance.
        
        Args:
            Kp (float): Proportional gain.
            Ki (float): Integral gain.
            Kd (float): Derivative gain.
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0
        self.last_error = 0

    def compute(self, error):
        """
        Computes the PID control output based on the current error.
        
        Args:
            error (float): The current error value.
        Returns:
            float: The control output based on the PID formula.
        """
        self.integral += error
        derivative = error - self.last_error
        self.last_error = error
        return (self.Kp * error + self.Ki * self.integral + self.Kd * derivative)

class WiFiConnection:
    """
    Class for managing the Wi-Fi connection to the ESP32.
    
    Attributes:
        ip (str): The IP address of the ESP32.
        port (int): The port number for the connection.
        sock (socket.socket): The socket object for the connection.
    """ 
    def __init__(self, ip, port):
        """ 
        Initializes a WiFiConnection instance.
        
        Args:
            ip (str): The IP address of the ESP32.
            port (int): The port number for the connection.
        """
        self.ip = ip
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(5)

        self.connect()

    def connect(self):
        """ Attempts to connect to the ESP32 over Wi-Fi. """
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(5)
            self.sock.connect((self.ip, self.port))
            self.sock.settimeout(0.05)  # Non-blocking or short timeout for recv()
            print("Connected to ESP32 over Wi-Fi.")
        except Exception as e:
            print("Wi-Fi socket connection failed:", e)
            sys.exit(1)

    def request_path(self, start, goal):
        """
        Requests a path from the ESP32 by sending the start and goal nodes.
        
        Args:
            start (str): The starting node identifier.
            goal (str): The goal node identifier.   
        
        Returns:
            list: The path received from the ESP32, or an empty list if no path is found.
        """
        try:
            self.sock.send((json.dumps({'start': start, 'goal': goal}) + '\n').encode())
        except Exception as e:
            print("Failed to send path request:", e)

        while True:
            try:
                line = self.sock.recv(1024).decode('utf-8').strip() # Read a line from the socket
                if line.startswith('{') and line.endswith('}'):
                    return json.loads(line).get('path', [])
            except (socket.timeout, UnicodeDecodeError, json.JSONDecodeError):
                continue # Ignore timeout and decoding errors, continue to receive data
            except Exception as e:
                print("Error receiving path:", e)
                break

    def close(self):
        """ Closes the Wi-Fi socket connection. """
        if self.sock:
            self.sock.close()

class RobotController:
    """
    Class for controlling the puck robot's movement along a predefined route.
    The controller takes the route and sets goals for the robot to follow.
    Goals are either the location of the next node or a turn to face the next node.
    The robot will follow the route, center itself at junctions, and turn as necessary.
    
    Attributes:
        puckRobot (PuckRobot): The puck robot instance to control.
        current_node (str): The current node the robot is at.
        route (list): The list of nodes representing the route to follow.
        step (int): The current step in the route.
        state (int): The current state of the robot (e.g., following, turning, finished).
        puckRobot_before_center (Position): The position of the robot before centering at a junction.
        next_node_pos (Position): The position of the next node in the route.
        goal_pos (Position): The goal position the robot is trying to reach.
    """
    def __init__(self, puckRobot, start_node):
        """
        Initializes a RobotController instance.
        
        Args:
            puckRobot (PuckRobot): The puck robot instance to control.
            start_node (str): The starting node identifier.
        """
        self.puckRobot = puckRobot

        self.current_node = start_node
        self.route = None
        self.step = 0

        self.state = STATE_FINISHED
        self.puckRobot_before_center = Position(0.0, 0.0, 0.0)

        self.next_node_pos = Position.from_node(start_node)
        self.goal_pos = Position.from_pos(puckRobot)

    def set_route(self, route):
        """ Sets the route for the robot to follow."""
        self.route = route
        self.step = 0
        self.set_next_goal()

    def set_next_goal(self):
        """ Sets the next goal position for the robot based on the current route."""
        print("Setting next goal")        
        # Check if the robot is at next_node
        if abs(self.next_node_pos.x - self.puckRobot.x) < 0.05 and abs(self.next_node_pos.y - self.puckRobot.y) < 0.05:
            # Check if there are more nodes to go to
            if (len(self.route) > self.step+1):
                self.step += 1
                self.next_node_pos = Position.from_node(self.route[self.step])
                print(f"Next node: {self.route[self.step]}, {self.next_node_pos}")
            else:
                print("Reached the end of the route.")
                print("state = STATE_FINISHED")
                self.state = STATE_FINISHED
                return
        
        
        # Get the direction for the next node
        if self.next_node_pos.x - self.puckRobot.x > 0.05:
            goal_phi = 0
        elif self.next_node_pos.x - self.puckRobot.x < -0.05:
            goal_phi = np.pi
        elif self.next_node_pos.y - self.puckRobot.y > 0.05:
            goal_phi = 0.5 * np.pi
        elif self.next_node_pos.y - self.puckRobot.y < -0.05:
            goal_phi = -0.5 * np.pi
        else:
            goal_phi = 0 #Already at goal, throw error
            print("Error: Robot is at goal position, but this should have been caught earlier in this function.")
                        
        # Check if the robot needs to rotate
        if abs(goal_phi - self.puckRobot.phi) > 0.05:
            print("Robot needs to turn")
            self.goal_pos = Position.from_pos(self.puckRobot)
            self.goal_pos.phi = goal_phi
            print(f"Next goal: {self.goal_pos}")
            print("state = STATE_TURN")
            self.state = STATE_TURN
        else:
            print("Robot is already facing the next node")
            print(self.next_node_pos)
            self.goal_pos = Position.from_pos(self.next_node_pos)
            print(f"Next goal: {self.goal_pos}")
            print("state = STATE_FOLLOW")
            self.state = STATE_FOLLOW
            

    def update(self):
        """ Updates the robot's state and movement based on the current position and sensors."""
        # Update robot position from encoders
        self.puckRobot.update_robot_pose()

        # Follow the line
        if self.state == STATE_FOLLOW:
            # Detect junction when gettings close to the junction
            if abs(self.goal_pos.x - self.puckRobot.x) < 0.05 and \
            abs(self.goal_pos.y - self.puckRobot.y) < 0.05 and \
            (self.puckRobot.line_left() or self.puckRobot.line_right()):
                self.puckRobot_before_center = Position.from_pos(self.puckRobot)
                self.state = STATE_CENTER_JUNCTION
                print("State = STATE_CENTER_JUNCTION")
                return

            if self.puckRobot.line_left() and not self.puckRobot.line_center() and not self.puckRobot.line_right():
                error = 1
            elif self.puckRobot.line_right() and not self.puckRobot.line_center() and not self.puckRobot.line_left():
                error = -1
            elif self.puckRobot.line_left() and self.puckRobot.line_center() and not self.puckRobot.line_right():
                error = 0.5
            elif self.puckRobot.line_right() and self.puckRobot.line_center() and not self.puckRobot.line_left():
                error = -0.5
            else:
                error = 0  # Either on the line, on a junction or off the line

            correction = pid_controller.compute(error)
            self.puckRobot.update_wheel_speeds(BASE_SPEED - correction, BASE_SPEED + correction)

        # When a junction is detected, the robot will continue to drive to for a short distance to center itself on the junction
        # When on the junction, the robot will updates its position to the junction location to avoid drifting over time
        elif self.state == STATE_CENTER_JUNCTION:
            if -0.1 < self.puckRobot.phi and self.puckRobot.phi < 0.1:
                if self.puckRobot.x > self.puckRobot_before_center.x + OFFSET_SENSOR_ROBOTCENTER:
                    self.puckRobot.x = self.goal_pos.x
                    self.puckRobot.y = self.goal_pos.y
                    self.puckRobot.phi = 0
                    self.set_next_goal()
                    return

            if abs(abs(self.puckRobot.phi) - np.pi) < 0.1:
                if self.puckRobot.x < self.puckRobot_before_center.x - OFFSET_SENSOR_ROBOTCENTER:
                    self.puckRobot.x = self.goal_pos.x
                    self.puckRobot.y = self.goal_pos.y
                    self.puckRobot.phi = np.pi
                    self.set_next_goal()
                    return

            if 0.5*np.pi-0.1 < self.puckRobot.phi and self.puckRobot.phi < 0.5*np.pi+0.1:
                if self.puckRobot.y > self.puckRobot_before_center.y + OFFSET_SENSOR_ROBOTCENTER:
                    self.puckRobot.x = self.goal_pos.x
                    self.puckRobot.y = self.goal_pos.y
                    self.puckRobot.phi = 0.5*np.pi
                    self.set_next_goal()
                    return

            if -0.5*np.pi-0.1 < self.puckRobot.phi and self.puckRobot.phi < -0.5*np.pi+0.1:
                if self.puckRobot.y < self.puckRobot_before_center.y - OFFSET_SENSOR_ROBOTCENTER:
                    self.puckRobot.x = self.goal_pos.x
                    self.puckRobot.y = self.goal_pos.y
                    self.puckRobot.phi = -0.5*np.pi
                    self.set_next_goal()
                    return

        # If the robot is in the turning state, it will turn until it is facing the next node
        elif self.state == STATE_TURN:
            diff = normalize_angle(self.goal_pos.phi - self.puckRobot.phi)
            if diff > 0.03:  
                self.puckRobot.update_wheel_speeds(-BASE_SPEED/2, BASE_SPEED/2)

            elif diff < -0.03:
                self.puckRobot.update_wheel_speeds(BASE_SPEED/2, -BASE_SPEED/2)
            
            else:
                self.set_next_goal()

        elif self.state == STATE_FINISHED:
            self.puckRobot.update_wheel_speeds(0, 0)
            return

# Initialize webots robot
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Initialize robot class
puckRobot = PuckRobot(robot, timestep, START_NODE, START_DIRECTION)

# Initialize Pid controller
pid_controller = PIDController(Kp, Ki, Kd)

# Initialize WiFi connection
sock = WiFiConnection(ESP32_IP, ESP32_PORT)

# Initialize robot controller
robot_controller = RobotController(puckRobot, START_NODE)

# Request path from ESP32
path = sock.request_path(START_NODE, GOAL_NODE)

if path:
    print("I am going to take this path: " + " -> ".join(path))
else:
    print("No path found. Exiting.")
    sys.exit(1)
if not path:
    print("No path found. Exiting.")
    sys.exit(1)

robot_controller.set_route(path)

# Main while loop
while robot.step(timestep) != -1:
    robot_controller.update()

sock.close()