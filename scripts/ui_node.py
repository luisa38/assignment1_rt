#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn
from turtlesim.msg import Pose

def ui_node():
    
    rospy.init_node('ui_node', anonymous=True)

    pub_turtle1 = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    pub_turtle2 = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)

    rospy.wait_for_service('/spawn')
    try:
        spawn_service = rospy.ServiceProxy('/spawn', Spawn)
        spawn_service(1.0, 1.0, 0.0, "turtle2")
        rospy.loginfo("Turtle2 criated with success!")
    except rospy.ServiceException as e:
    #ver pq não apareceeeeeee
        rospy.logerr(f"Error criating turtle2: {e}")
        return

    while not rospy.is_shutdown():
        #asked turtle
        print("\nSelect the turtle to control:")
        print("1 - Turtle1")
        print("2 - Turtle2")
        choice = input("Choose (1/2): ")

        if choice not in ['1', '2']:
            print("Invalid Option! Try again.")
            continue

        #ask for velocities 
        try:
            linear_vel = float(input("Insert linear velocity: "))
            angular_vel = float(input("Insert angular velocity: "))
        except ValueError:
            print("Invalid! Try again.")
            continue

        #create velocities
        vel = Twist()
        vel.linear.x = linear_vel
        vel.angular.z = angular_vel

        #send commands to the chosen turtle
        if choice == '1':
            pub_turtle1.publish(vel)
        else:
            pub_turtle2.publish(vel)

        rospy.loginfo("Turtle will move for 1 second.")
        #mexe 1 segundo
        rospy.sleep(1)

        #stop turtle
        vel.linear.x = 0
        vel.angular.z = 0
        if choice == '1':
            pub_turtle1.publish(vel)
        else:
            pub_turtle2.publish(vel)
        rospy.loginfo("Turtle stopped.")

if __name__ == '__main__':
    try:
        ui_node()
    except rospy.ROSInterruptException:
        pass

