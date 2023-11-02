#!/usr/bin/env python3

########## FILL HERE ##########
# NAME & SURNAME: TANAY BENSU YURTTURK
# STUDENT ID: 150220766
###############################

# Even though I don't understand why, 
# sometimes the turtle does not go to the desired point 
# or does not rotate desired angle and even if the first 
# square is not drawn perfectly, it gets better later. 
# It may be better to run the code a few times and evaluate.


########## DO NOT EDIT THESE LIBRARIES ##########
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
#################################################

########## ADD YOUR LIBRARIES HERE ##########
import math
import time
###########################################
##########################################
class TurtleCleaner(Node):
    def __init__(self, points):
        """
            Cleans the given area with the turtle

            Parameters:
                points (list): List of points that define the area
            Returns:
                None
        """

        ########## DO NOT EDIT ANYTHING HERE ##########
        # Initialize the node
        super().__init__('turtle_cleaner')
        self.area = points

        # Create the Twist publisher and Pose subscriber
        self.twist_publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10) #to move the turtle
        self.turtle_pose_sub_ = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10) #turtle's pose to track its current position
        self.turtle_pose_sub_ 
        ###############################################

        ########## ADD YOUR CODE HERE ##########
        self.current_pose = Pose()
        self.msg_twist = Twist()
        self.clear_area(self.area)
        
        ########################################

    def pose_callback(self, msg): #function to handle the turtle's position updates and using this information to make navigation decisions.
        """
            Callback function for the Pose subscriber.

            Parameters:
                msg (Pose): Current turtle pose
            Returns:
                Noneself.msg_twist
        """
        ########## ADD YOUR CODE HERE ##########
        self.current_pose = msg 
        pass
        ########################################

    def move(self, distance, speed, isForward):
        """
            Moves the turtle with the given speed for the given distance

            Parameters: 
                distance (float): Distance to move
                speed (float): Speed to move
                isForward (bool): Direction of the movement (True: Forward, False: Backward)

            Returns:
                None
        """
        ########## ADD YOUR CODE HERE ##########
        
        # First I have to ensure that angular and other direction movements are stopped
        self.msg_twist.linear.y=0.0
        self.msg_twist.linear.z=0.0
        self.msg_twist.angular.x=0.0
        self.msg_twist.angular.y=0.0
        self.msg_twist.angular.z=0.0
        
        # Receive the Pose information
        rclpy.spin_once(self)

        # Setting the initial positions
        start_x = self.current_pose.x
        start_y = self.current_pose.y

         # Setting linear velocity based on direction
        if isForward:
            self.msg_twist.linear.x = abs(float(speed))
        else:
            self.msg_twist.linear.x = -abs(float(speed))

        # Initialize the start time
        start_time = time.time()

        # Calculate the time needed for the movement operation
        time_limit = (distance / speed)*100 # t = x / v

        # Until time limit is reached, turtle should move by publishing twist msg
        while self.calculate_elapsed_time(start_time)<time_limit and (self.distance_cal(start_x,start_y)<=distance):
            self.twist_publisher_ .publish(self.msg_twist)
            #self.get_logger().info(f"\n~~~~~~~~~~~~~~~~~~~~~where turtle : {self.current_pose.x,self.current_pose.y}")
            rclpy.spin_once(self) # to get new callback in each iteration
        
        # Stop the turtle after reaching the desired distance
        self.msg_twist.linear.x = 0.0
        self.twist_publisher_ .publish(self.msg_twist)
        pass
        ########################################

    def rotate(self, angle, speed, isClockwise):
        """
            Rotates the turtle with the given speed for the given angle

            Parameters:
                angle (float): Angle to rotate in degrees
                speed (float): Speed to rotate
                isClockwise (bool): Direction of the rotation (True: Clockwise, False: Counter Clockwise)

        """
        ########## ADD YOUR CODE HERE ##########
        
        
        # First I have to ensure that linear and other angular movements are stopped
        self.msg_twist.linear.x=0.0
        self.msg_twist.linear.y=0.0
        self.msg_twist.linear.z=0.0
        self.msg_twist.angular.x=0.0
        self.msg_twist.angular.y=0.0

        while angle<0 : angle+=360
        while angle>360 : angle-=360

        # Manuel Normalization
        if angle > 180 and not isClockwise: 
            angle = 360.0-angle 
            isClockwise = True
        
        if angle > 180 and isClockwise: 
            angle = 360.0-angle 
            isClockwise = False

        # To prevent infinite loop
        if(angle == 180):
            angle = angle - 1.0 

        #self.get_logger().info(f"\n~~~~~~~~~~~~~~~~~~~~~angle : {angle}")
        rclpy.spin_once(self)  # Receive the Pose information


        # Initialize the start time 
        start_time = time.time()
        # Calculation of time to complete the rotation
        time_limit = (math.radians(angle) / math.radians(speed))*100 # Rotation Angle (in radians)=Angular Speed (in radians per second)×Time (in seconds)

        # Setting angular velocity based on rotation direction
        if isClockwise: 
            self.msg_twist.angular.z = -abs(math.radians(speed))  # z axis is the direction of rotation

        else: # z is positive in counter clockwise direction
            self.msg_twist.angular.z = abs(math.radians(speed))


        # Rotation until time limit is reached and angle is completed by publishing twist msg
        while self.calculate_elapsed_time(start_time)<=(time_limit):
            self.twist_publisher_.publish(self.msg_twist)
            #self.get_logger().info(f"\n~~~~~~~~~~~~~~~~~~~~~ elapsed time, time limit : {self.calculate_elapsed_time(start_time),time_limit}") 
            #self.get_logger().info(f"\n~~~~~~~~~~~~~~~~~~~~~ theta : {math.degrees(self.current_pose.theta)}")
            rclpy.spin_once(self)
        
        
        # Stop the turtle after reaching the desired rotation angle
        self.msg_twist.angular.z = 0.0
        self.twist_publisher_.publish(self.msg_twist)
        pass
        ########################################

    def go_to_a_goal(self, point, linear_speed, angular_speed):
        """
            Moves the turtle to the given point with the given speeds

            Parameters:
                point (list): Point to go
                linear_speed (float): Speed to move
                angular_speed (float): Speed to rotate in degrees

            Returns:
                None
        """
        ########## ADD YOUR CODE HERE ##########
        # To make the turtle go to a specific point, I have to calculate the linear and angular velocities based on the difference between the current pose and the target point.
        
        ##rclpy.spin_once(self) # to get the correct pose information by processing messages  
        ####self.get_logger().info(f"\n\n\n\n~~~~~~~~~~~~~~~~~~~~~points : {point}")
        
        # Get the needed pose and point informations
        target_x = point[0]
        target_y = point[1]
        start_x = self.current_pose.x 
        start_y = self.current_pose.y
        #self.get_logger().info(f"\n\n\n\n~~~~~~~~~~~~~~~~~~~~~ targetx: {target_x}, targety: {target_y}, startx: {start_x},  starty: {start_y}, theta: {self.current_pose.theta}")


        # Rotation Part
        rclpy.spin_once(self)  # Receive the Pose information
        # Calculating the rotation angle to rotate the turtle towards the point
        Qt = math.degrees(math.atan2((target_y-start_y),(target_x-start_x))) # Desired Point angle
        Q = math.degrees(self.current_pose.theta) # Current angle

        #self.get_logger().info(f"\n\n\n\n\n~~~~~~~~~~~~~~~~~~Qt : {Qt}")
        #self.get_logger().info(f"\n~~~~~~~~~~~~~~~~~~~Q : {Q}")
        
        if Q<=0: # bottom half of the plane
            angle = (abs(Q)+Qt) # this is the angle that robot should turn to get its face to the target
            #self.get_logger().info(f"\n~~~~~~~~~~~~~~~~~~~~~ angleeeeeeeee : {angle}")
            self.rotate(angle,angular_speed,False)
        
        else: # top half of the plane
            angle = (Q-Qt) # this is the angle that robot should turn to get its face to the target
            #self.get_logger().info(f"\n~~~~~~~~~~~~~~~~~~~~~angleeeeeeeee : {angle}")
            self.rotate(angle,angular_speed,True)
        
        # Movement Part
        distance = self.distance_cal(target_x,target_y)
        self.move(distance,linear_speed,True)
        pass
        ########################################
    
    ########## YOU CAN ADD YOUR FUNCTIONS HERE ##########
    
    def distance_cal(self,x,y):
        return math.sqrt(pow((x - self.current_pose.x), 2) + pow((y - self.current_pose.y), 2))
    

    def clear_area(self,points):
        # find the points
        #rclpy.spin_once(self) # to get the correct pose information by processing messages  
        
        # Determine the corners of the square
        x_min = min(p[0] for p in points)
        x_max = max(p[0] for p in points)
        y_min = min(p[1] for p in points)
        y_max = max(p[1] for p in points)
        

        linear_speed = 6.0
        angular_speed = 170.0 # (degrees/sec)
        shrink = 0.1 # the amount that will shrink in at each iteration
        enlarge = 0.1 # the amount that will enlargement in at each iteration

        point_x_max = x_max
        point_y_max = y_max
        
        point_x_min = x_min
        point_y_min = y_min


        # First go to the right top corner
        self.go_to_a_goal([point_x_max,point_y_max],linear_speed,angular_speed) # 7,7 -> going to the right top corner for the first movement,since firts move I make have to one of the 4 points
        
        while True:
            self.firstLoop(points,shrink,point_x_max,point_x_min,point_y_max,point_y_min,linear_speed,angular_speed)
            self.secondLoop()

        

    def firstLoop(self,points,shrink,point_x_max,point_x_min,point_y_max,point_y_min,linear_speed,angular_speed):
        # First loop to make turtle to go towards middle of the square
        while self.is_inside_target_area(points, self.current_pose.x, self.current_pose.y):
            self.go_to_a_goal([point_x_max,point_y_min],linear_speed,angular_speed) # 7,4
            self.go_to_a_goal([point_x_min,point_y_min],linear_speed,angular_speed) # 4,4
            self.go_to_a_goal([point_x_min,point_y_max],linear_speed,angular_speed) # 4,7
            point_x_max = point_x_max-shrink
            self.go_to_a_goal([point_x_max,point_y_max],linear_speed,angular_speed)
            point_y_max = point_y_max-shrink
            point_y_min = point_y_min+shrink
            point_x_min = point_x_min+shrink
            rclpy.spin_once(self)

    def secondLoop(self,points,enlarge,point_x_max,point_x_min,point_y_max,point_y_min,linear_speed,angular_speed):
        # Second loop to make turtle to go towards corners of the square
        while self.is_inside_target_area(points, self.current_pose.x, self.current_pose.y):
            self.go_to_a_goal([point_x_max,point_y_min],linear_speed,angular_speed) 
            self.go_to_a_goal([point_x_min,point_y_min],linear_speed,angular_speed) 
            self.go_to_a_goal([point_x_min,point_y_max],linear_speed,angular_speed)
            point_x_max = point_x_max+enlarge
            self.go_to_a_goal([point_x_max,point_y_max],linear_speed,angular_speed)
            point_y_max = point_y_max+enlarge
            point_y_min = point_y_min-enlarge
            point_x_min = point_x_min-enlarge
            rclpy.spin_once(self)
          
    # Check if the current turtles position is inside the given area.
    def is_inside_target_area(self, points, x, y, margin=0.3):
        min_x = min(p[0] for p in points) - margin
        max_x = max(p[0] for p in points) + margin
        min_y = min(p[1] for p in points) - margin
        max_y = max(p[1] for p in points) + margin
        return (x >= min_x and x <= max_x and y >= min_y and y <= max_y)    
    
    #  Calculate the elapsed time since the start of the movement or rotation.
    def calculate_elapsed_time(self, start_time):    
        return (time.time() - start_time)*100
    #####################################################


########## DO NOT EDIT ANYTHING BELOW THIS LINE ##########
def main(args=None):
    rclpy.init(args=args)
    points =[[7.0, 7.0], [7.0, 4.0], [4.0, 4.0], [4.0, 7.0]]
    turtle_cleaner = TurtleCleaner(points)
    rclpy.spin(turtle_cleaner)
    rclpy.shutdown()

if __name__ == '__main__':
    main()