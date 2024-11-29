#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float32.h"
#include <cmath>

// definir os limites
#define X_MAX 10.0
#define Y_MAX 10.0
#define X_MIN 1.0
#define Y_MIN 1.0
// dist. tartarugas
#define DISTANCE_THRESHOLD 1.5 

double x1, y1_turtle, theta1, x2, y2_turtle, theta2;

// posição da turtle1
void turtle1PoseCallback(const turtlesim::Pose::ConstPtr& msg) {
    x1 = msg->x;
    y1_turtle = msg->y;
    theta1 = msg->theta;
    ROS_INFO("Turtle1 position: [%f, %f, %f]", x1, y1_turtle, theta1);
}

// posição da turtle2
void turtle2PoseCallback(const turtlesim::Pose::ConstPtr& msg) {
    x2 = msg->x;
    y2_turtle = msg->y;
    theta2 = msg->theta;
    ROS_INFO("Turtle2 position: [%f, %f, %f]", x2, y2_turtle, theta2);
}

// distância entre as tartarugas
double calculateDistance() {
    double dx = x1 - x2;
    double dy = y1_turtle - y2_turtle;
    return sqrt(dx * dx + dy * dy);
}

// ver se uma tartaruga está fora dos limites
bool isOutOfBounds(double x, double y) {
    return (x < X_MIN || x > X_MAX || y < Y_MIN || y > Y_MAX);
}

//parar tartaruga
void stopTurtle(ros::Publisher& pub) {
    geometry_msgs::Twist stop_msg;
    pub.publish(stop_msg); 
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "distance_node");
    ros::NodeHandle nh;

    ros::Subscriber turtle1_sub = nh.subscribe("turtle1/pose", 1, turtle1PoseCallback);
    ros::Subscriber turtle2_sub = nh.subscribe("turtle2/pose", 1, turtle2PoseCallback);

    ros::Publisher turtle1_pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);
    ros::Publisher turtle2_pub = nh.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 1);

    ros::Publisher distance_pub = nh.advertise<std_msgs::Float32>("turtles_distance", 1);

    ros::Rate rate(10); // meter isto - corre 10x por seg.

    while (ros::ok()) {
        ros::spinOnce();
        
        //distância entre as tartarugas
        double distance = calculateDistance();
        std_msgs::Float32 distance_msg;
        distance_msg.data = distance;
        distance_pub.publish(distance_msg);

        ROS_INFO("Distance between turtles: %f", distance);

        //verificar se as tartarugas estão muito próximas
        if (distance < DISTANCE_THRESHOLD) {
            ROS_WARN("Turtles too close! Stopping turtle.");
            stopTurtle(turtle2_pub);
            stopTurtle(turtle1_pub);
        }

        //verificar se alguma tartaruga está fora dos limites
        if (isOutOfBounds(x1, y1_turtle)) {
            ROS_WARN("Turtle1 is out of bounds. Stopping.");
            stopTurtle(turtle1_pub);  // Parar turtle1
        }
        if (isOutOfBounds(x2, y2_turtle)) {
            ROS_WARN("Turtle2 is out of bounds. Stopping.");
            stopTurtle(turtle2_pub);  // Parar turtle2
        }

        rate.sleep();
    }

    return 0;
}


