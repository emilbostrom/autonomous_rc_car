#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

void positionSubscriber(const geometry_msgs::Pose pos) {
    ROS_INFO("I heard: [%s]", pos->data.c_str());
}

int main(int argc, char** argv) {

    ros::init(argc,argv, "listener");
    ros::Nodehandle n;

    ros::Subscriber sub = n.subscribe("chatter",1000,positionSubscriber);

    ros::spin();
    
    return 0;

}