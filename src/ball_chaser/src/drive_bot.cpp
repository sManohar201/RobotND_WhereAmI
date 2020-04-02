#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

#include "ball_chaser/DriveToTarget.h"


class DriveBot
{

private:
// create nodehandle object
    ros::NodeHandle n_;
// publisher object
    ros::Publisher pub_;
    // loop rate
    ros::Rate r_;
    // service object
    ros::ServiceServer ser_;

public:
    DriveBot(int rate);
    ~DriveBot(){};
    bool callBack(ball_chaser::DriveToTarget::Request &req, ball_chaser::DriveToTarget::Response &res);
};

DriveBot::DriveBot(int rate) : r_(rate)
{
    // create a publisher object to geometry_msgs::Twist type
    pub_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    // create a service object
    ser_ = n_.advertiseService("/drive_robot/command", &DriveBot::callBack, this);
}

bool DriveBot::callBack(ball_chaser::DriveToTarget::Request &req, ball_chaser::DriveToTarget::Response &res) {
    ROS_INFO("Go to Position request - Linear x: %1.2f, Angular z: %1.2f", (float)req.linear_x, (float)req.angular_z);
    geometry_msgs::Twist motor_command;
    motor_command.linear.x = req.linear_x;
    motor_command.angular.z = req.angular_z;
    pub_.publish(motor_command);

    // sleep for 0.1 second
    r_.sleep();

    res.msg_feedback = "Position set at" + std::to_string(req.linear_x) + "Angle set at" + std::to_string(req.angular_z);
    ROS_INFO_STREAM(res.msg_feedback);

    return true;
}

int main(int argc, char ** argv) {
    // Initialize a ROS node
    ros::init(argc, argv, "drive_bot");
    // Create a ROS Nodehandle object
    ros::NodeHandle n;
    // initialize drive bot object
    DriveBot drive_bot(10);

    ros::spin();

    return 0;
}