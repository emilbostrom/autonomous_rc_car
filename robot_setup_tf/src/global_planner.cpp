#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

void positionSubscriber(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    ROS_INFO_STREAM("Received pose: " << msg);
    x_current = msg->pose.position.x;
    y_current = msg->pose.position.y;
    z_current = msg->pose.position.z;
    x_quat = msg->pose.orientation.x;
    y_quat = msg->pose.orientation.y;
    z_quat = msg->pose.orientation.z;
    w_quat = msg->pose.orientation.w;

    ROS_INFO_STREAM(x_current);
    ROS_INFO_STREAM(y_current);
    ROS_INFO_STREAM(z_current);
    ROS_INFO_STREAM(x_quat);
    ROS_INFO_STREAM(y_quat);
    ROS_INFO_STREAM(z_quat);
    ROS_INFO_STREAM(w_quat);
}

int main(int argc, char** argv) {

    ros::init(argc,argv, "listener");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("chatter",1000,positionSubscriber);

    ros::spin();
    
    return 0;

}