#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time
from std_srvs.srv import Empty

def poseCallback(pose_message):
    global x
    global y, yaw
    x= pose_message.x
    y= pose_message.y
    yaw = pose_message.theta

    #print "pose callback"
    #print ('x = {}'.format(pose_message.x)) #new in python 3
    #print ('y = %f' %pose_message.y) #used in python 2
    #print ('yaw = {}'.format(pose_message.theta)) #new in python 3


def move(velocity_publisher, speed, distance, is_forward):
        #declare a Twist message to send velocity commands
        velocity_message = Twist()
        #get current location 
        global x, y
        x0=x
        y0=y

        if (is_forward):
            velocity_message.linear.x =abs(speed)
        else:
        	velocity_message.linear.x =-abs(speed)

        distance_moved = 0.0
        loop_rate = rospy.Rate(10) # we publish the velocity at 10 Hz (10 times a second)    
        
        while True :
                rospy.loginfo("Turtlesim moves forwards")
                velocity_publisher.publish(velocity_message)

                loop_rate.sleep()
                
                distance_moved = abs(math.sqrt(((x-x0) ** 2) + ((y-y0) ** 2)))
                print  (distance_moved)
                print(x)
                if  not (distance_moved<distance):
                    rospy.loginfo("reached")
                    break
        
        #finally, stop the robot when the distance is moved
        velocity_message.linear.x =0
        velocity_publisher.publish(velocity_message)
    
def rotate (velocity_publisher, angular_speed_degree, relative_angle_degree, clockwise):
    
    velocity_message = Twist()

    angular_speed=math.radians(abs(angular_speed_degree))

    if (clockwise):
        velocity_message.angular.z =-abs(angular_speed)
    else:
        velocity_message.angular.z =abs(angular_speed)

    angle_moved = 0.0
    loop_rate = rospy.Rate(10) # we publish the velocity at 10 Hz (10 times a second)    
    cmd_vel_topic='/turtle1/cmd_vel'
    velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

    t0 = rospy.Time.now().to_sec()

    while True :
        rospy.loginfo("Turtlesim rotates")
        velocity_publisher.publish(velocity_message)

        t1 = rospy.Time.now().to_sec()
        current_angle_degree = (t1-t0)*angular_speed_degree
        loop_rate.sleep()


                       
        if  (current_angle_degree>relative_angle_degree):
            rospy.loginfo("reached")
            break

    #finally, stop the robot when the distance is moved
    velocity_message.angular.z =0
    velocity_publisher.publish(velocity_message)


def go_to_goal(velocity_publisher, x_goal, y_goal):
    global x
    global y, yaw

    velocity_message = Twist()

    while (True):
        K_linear = 0.5 
        distance = abs(math.sqrt(((x_goal-x) ** 2) + ((y_goal-y) ** 2)))

        linear_speed = distance * K_linear

        K_angular = 4.0
        desired_angle_goal = math.atan2(y_goal-y, x_goal-x)
        angular_speed = (desired_angle_goal-yaw)*K_angular

        velocity_message.linear.x = linear_speed
        velocity_message.angular.z = angular_speed

        velocity_publisher.publish(velocity_message)
        print ('x=', x, ', y=',y, ', distance to goal: ', distance)

        if (distance <0.01):
            break

def setDesiredOrientation(publisher, speed_in_degree, desired_angle_degree):
    relative_angle_radians = math.radians(desired_angle_degree) - yaw
    clockwise=0
    if relative_angle_radians < 0:
        clockwise = 1
    else:
        clockwise = 0
    print ("relative_angle_radians: ",math.degrees(relative_angle_radians))
    print ("desired_angle_degree: ",desired_angle_degree)
    rotate(publisher, speed_in_degree,math.degrees(abs(relative_angle_radians)), clockwise)

def gridClean(publisher):
 
    desired_pose = Pose()
    desired_pose.x = 1
    desired_pose.y = 1
    desired_pose.theta = 0
 
    go_to_goal(publisher, 1,1)
 
    setDesiredOrientation(publisher, 30, math.radians(desired_pose.theta))
 
    for i in range(5):
        move(publisher, 2.0, 1.0, True)
        rotate(publisher, 20, 90, False)
        move(publisher, 2.0, 9.0, True)
        rotate(publisher, 20, 90, True)
        move(publisher, 2.0, 1.0, True)
        rotate(publisher, 20, 90, True)
        move(publisher, 2.0, 9.0, True)
        rotate(publisher, 20, 90, False)
    pass

def squareMotion(velocity_publisher,  angular_speed, linear_speed):
    while True:
        try:
            side_length = float(input("Enter the side length of the square (in meters): "))
            if side_length > 10.5:
                print("Invalid side length! The side length must be 10.5 meters or less to stay within the grid.")
            elif side_length <= 0:
                print("Invalid side length! The side length must be greater than 0.")
            else:
                break  # Valid input, exit the loop
        except ValueError:
            print("Invalid input! Please enter a numeric value for the side length.")

    # Execute square motion
    for _ in range(4):
        move(velocity_publisher, linear_speed, side_length, True)
        rotate(velocity_publisher, angular_speed, 90, True)

def triangleMotion(velocity_publisher, side_length):
    for _ in range(3):  # Loop to create three sides of the triangle
        # Move forward for the length of the triangle's side
        move(velocity_publisher, speed=1.0, distance=side_length, is_forward=True)
        # Rotate 120 degrees to the left
        rotate(velocity_publisher, angular_speed_degree=30, relative_angle_degree=120, clockwise=False)

def circularMotion(raduis):
    vel_msg = Twist()
    loop_rate = rospy.Rate(10)
    while 1: 
        vel_msg.linear.x = raduis
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 1.5
        loop_rate.sleep()
        velocity_publisher.publish(vel_msg)

def spiralMotion(ratechange):
    vel_msg = Twist()
    loop_rate = rospy.Rate(1)
    rk = 0
    print (x)
    print (y)
    while x < 8 and y < 8:
        rk=rk+ratechange
        print (x)
        print (y)
        vel_msg.linear.x =rk
        vel_msg.linear.y =0
        vel_msg.linear.z =0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 4
        velocity_publisher.publish(vel_msg)
        loop_rate.sleep()
 
    vel_msg.linear.x = 0
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)




def zigzag(velocity_publisher,side_length, count):
   while True:
        try:
            side_length = float(input("Enter the side length of the square (in meters): "))
            if side_length > 11:
                print("Invalid side length! The side length must be 11 meters or less to stay within the grid.")
            elif side_length <= 0:
                print("Invalid side length! The side length must be greater than 0.")
            else:
                break  # Valid input, exit the loop
        except ValueError:
            print("Invalid input! Please enter a numeric value for the side length.")
   for _ in range(count):
        move(velocity_publisher, 1.0, side_length, True)
        rotate(velocity_publisher, 30, 120, False)
        move(velocity_publisher, 1.0, side_length, True)
        rotate(velocity_publisher, 30, 120, True)

def spiralClean(velocity_publisher, wk, rk):
    vel_msg = Twist()
    loop_rate = rospy.Rate(1)
 
    while((x<10.5) and (y<10.5)):
        rk=rk+1
        vel_msg.linear.x =rk
        vel_msg.linear.y =0
        vel_msg.linear.z =0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z =wk
        velocity_publisher.publish(vel_msg)
        loop_rate.sleep()
 
    vel_msg.linear.x = 0
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)

if __name__ == '__main__':
    try:
        print("hello");
        rospy.init_node('turtlesim_motion_pose', anonymous=True)

        # Initialize global variables
        global x, y, yaw
        x = 0.0
        y = 0.0
        yaw = 0.0

        # Declare velocity publisher
        cmd_vel_topic = '/turtle1/cmd_vel'
        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

        # Subscriber for pose
        position_topic = "/turtle1/pose"
        rospy.Subscriber(position_topic, Pose, poseCallback, queue_size=10)

        # Ensure data is received before continuing
        while x == 0.0 and y == 0.0:
            rospy.loginfo("Waiting for pose data...")
            time.sleep(1)

        # Display choices once data is ready
        while True:
            print("\nSelect one of the following motion trajectories for the turtle robot:")
            print("1. Square")
            print("2. Triangle")
            print("3. Circular")
            print("4. Spiral")
            print("5. Point to Point")
            print("6. Zigzag motion")
            print("7. Exit")

            try:
                choice = int(input("Enter your choice (1-7): "))

                if choice == 1:
                    linear_speed = 1.0
                    angular_speed = 30
                    squareMotion(velocity_publisher, angular_speed, linear_speed)
                elif choice == 2:
                    print("You selected: Triangle motion")
                    side_length = 2.0  # Specify the side length of the triangle
                    triangleMotion(velocity_publisher, side_length)
                elif choice == 3:
                    print("You selected: Circular motion")
                    try:
                        radius = float(input("Enter the radius of the circle: "))
                        if radius <= 0:
                            print("Invalid radius! The radius must be greater than 0.")
                        else:
                            circularMotion(radius)
                    except ValueError:
                        print("Invalid input! Please enter a valid number for the radius.")

                elif choice == 4:
                    c = float(input("Enter rate of change for spiral: "))
                    spiralMotion(c)
                elif choice == 5:
                    print("You selected: Point to Point motion")
                elif choice == 6:
                    side_length = float(input("Enter the side_length"))
                    count = int(input("Enter the count"))
                    zigzag(velocity_publisher, side_length, count)
                elif choice == 7:
                    print("Exiting the program. Goodbye!")
                    break
                else:
                    print("Invalid choice. Please enter a number between 1 and 7.")
            except ValueError:
                print("Invalid input. Please enter a number between 1 and 7.")

    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated.")
