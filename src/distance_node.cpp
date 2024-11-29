#include "ros/ros.h"
#include <unistd.h>
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float32.h"
#include <cmath>

// definir os limites
#define X_MAX 10.0
#define Y_MAX 10.0
#define X_MIN 1.0
#define Y_MIN 1.0

#define DISTANCE_THRESHOLD 1.5

double x1, y1_turtle, x2, y2_turtle, theta1, theta2;

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

// se a vel. linear for mt rápida as tartarugas podem ficar presas nos limitesssss
// corrigir diretamente a posição da tartaruga
// ver se funciona direito
void correctTurtlePosition(ros::Publisher& pub, double& x, double& y) {
    geometry_msgs::Twist correction_msg;
    correction_msg.linear.x = 0; // parar movimento em X
    correction_msg.linear.y = 0; // parar movimento em XY
    correction_msg.angular.z = 0; // parar rotações

    // ajustar posição X para os limites 
    if (x < X_MIN) {
        x = X_MIN;
        ROS_WARN("Turtle at x < X_MIN. Correcting to x = %.1f", X_MIN);
    } else if (x > X_MAX) {
        x = X_MAX;
        ROS_WARN("Turtle at x > X_MAX. Correcting to x = %.1f", X_MAX);
    }

    // ajustar posição Y para os limites
    if (y < Y_MIN) {
        y = Y_MIN;
        ROS_WARN("Turtle at y < Y_MIN. Correcting to y = %.1f", Y_MIN);
    } else if (y > Y_MAX) {
        y = Y_MAX;
        ROS_WARN("Turtle at y > Y_MAX. Correcting to y = %.1f", Y_MAX);
    }

    pub.publish(correction_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "turtlebot_subscriber");
    ros::NodeHandle nh;

    ros::Subscriber turtle1_sub = nh.subscribe("turtle1/pose", 1, turtle1PoseCallback);
    ros::Subscriber turtle2_sub = nh.subscribe("turtle2/pose", 1, turtle2PoseCallback);

    ros::Publisher turtle1_pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);
    ros::Publisher turtle2_pub = nh.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 1);

    ros::Rate rate(10); // meter isto - corre 10x por seg.
    while (ros::ok()) {
        ros::spinOnce();

        //distância entre as tartarugas
        double distance = calculateDistance();
        ROS_INFO("Distance between turtles: %f", distance);

        //verificar e corrigir posição da turtle1
        if (x1 < X_MIN || x1 > X_MAX || y1_turtle < Y_MIN || y1_turtle > Y_MAX) {
            correctTurtlePosition(turtle1_pub, x1, y1_turtle);
        }

        //verificar e corrigir posição da turtle2
        if (x2 < X_MIN || x2 > X_MAX || y2_turtle < Y_MIN || y2_turtle > Y_MAX) {
            correctTurtlePosition(turtle2_pub, x2, y2_turtle);
        }

        //verificar se estão muito próximas
        if (distance < DISTANCE_THRESHOLD) {
            ROS_WARN("Turtles too close! Stopping turtle.");
            geometry_msgs::Twist stop_msg;
            // PI PI PI PAROU
            turtle1_pub.publish(stop_msg); 
            turtle2_pub.publish(stop_msg); 
        }

        rate.sleep();
    }

    return 0;
}


