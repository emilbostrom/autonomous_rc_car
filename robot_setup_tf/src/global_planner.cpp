#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sstream>
#include <vector>
#include <string>
#include <iostream>
#include <cmath>
#include <random>


double xGoal = 5;
double yGoal = 5;
double z_goal = 0;
double x_quat_goal = 0.707;
double y_quat_goal = 0;
double z_quat_goal = 0;
double w_quat_goal = 0.707;



std::vector<geometry_msgs::PoseStamped::ConstPtr> pose;

class Node{
    public:
        int x;
        int y;
        int id;
        int idParent;
        int cost;
        
        Node(int x, int y, int id){
            ROS_INFO_STREAM("Node id: " << id);
            ROS_INFO_STREAM("Width generated: " << x);
            ROS_INFO_STREAM("Height generated: " << y);
        }

        double calcDistance(double x1, double y1, double x2, double y2) {
            return sqrt(pow((x2-x1),2) + pow((y2-y1),2));
        }

        Node FindNearestNode(std::vector<Node> Tree) {
            Node nearestNode = Tree[0];
            ROS_INFO_STREAM("Start node info: " << nearestNode.id << " x: " << nearestNode.x << " y: " << nearestNode.y);
            ROS_INFO_STREAM("Should be same as above: " << Tree[0]);

            double nearestDist = calcDistance(x,y,Tree[0].x,Tree[0].y);
            ROS_INFO_STREAM("Distance between node and start node: " << nearestDist);

            ROS_INFO_STREAM("Tree size: " << Tree.size());

            for(int i = 1; i < Tree.size(); i++) {
                double dist = calcDistance(x,y,Tree[i].x,Tree[i].y);
                if (dist < nearestDist) {
                    nearestNode = Tree[i];
                    nearestDist = dist;
                }
            }
            ROS_INFO_STREAM("Closest to node " << id << " is node: " << nearestNode.id << " with dist: " << nearestDist);
            return nearestNode;
        }

};


class GlobalPlanner{
    public:
        std::string frameIdMap = "map";
        double mapResolution; // [m/cell]
        int mapWidth; // [m]
        int mapHeight; // [m]
        double stepLength; // [m]
        double goalDistThreshold; // [m]

        double xCurrent;
        double yCurrent;
        double zCurrent;
        double xQuat;
        double yQuat;
        double zQuat;
        double wQuat;

        typedef std::mt19937 MyRNG;
        MyRNG rng;

        std::uniform_int_distribution<uint32_t> widthGenerator;
        std::uniform_int_distribution<uint32_t> heightGenerator;

        GlobalPlanner(const geometry_msgs::PoseStamped::ConstPtr& poseMsg, 
                      const nav_msgs::MapMetaData::ConstPtr& mapMetaMsg) {
            
            ROS_INFO_STREAM("Received pose: " << poseMsg);
            xCurrent = poseMsg->pose.position.x;
            yCurrent = poseMsg->pose.position.y;
            zCurrent = poseMsg->pose.position.z;
            xQuat = poseMsg->pose.orientation.x;
            yQuat = poseMsg->pose.orientation.y;
            zQuat = poseMsg->pose.orientation.z;
            wQuat = poseMsg->pose.orientation.w;

            mapResolution = mapMetaMsg->resolution;
            mapWidth = mapMetaMsg->width*mapResolution;
            mapHeight = mapMetaMsg->height*mapResolution;

            ROS_INFO_STREAM("Map resolution: " << mapResolution);
            ROS_INFO_STREAM("Map width: " << mapWidth);
            ROS_INFO_STREAM("Map height: " << mapHeight);
            
            uint32_t seed_val = 100;
            rng.seed(seed_val);
            widthGenerator = std::uniform_int_distribution<uint32_t>(0, mapWidth);
            heightGenerator = std::uniform_int_distribution<uint32_t>(0, mapHeight);

            stepLength = mapResolution*2;
            goalDistThreshold = stepLength*2;
        }

        double calcDistance(double x1, double y1, double x2, double y2) {
            return sqrt(pow((x2-x1),2) + pow((y2-y1),2));
        }

        const nav_msgs::Path createPath(){
            
            nav_msgs::Path path;
            std::vector<geometry_msgs::PoseStamped> posesStampedVectorMsg;

            geometry_msgs::PoseStamped poseStampedMsg;
            poseStampedMsg.header.frame_id = frameIdMap;

            geometry_msgs::Pose pose;

            double xPathPos = xCurrent;
            double yPathPos = yCurrent;

            double distToGoal = calcDistance(xPathPos,yPathPos,xGoal,yGoal);


            // Create first node, which is current position
            Node nodeOrigin(xCurrent,yCurrent,0);
            nodeOrigin.idParent = 0;
            nodeOrigin.cost = 0;

            // Create an array of all nodes
            std::vector<Node> Tree;
            Tree.push_back(nodeOrigin);

            int maxIterationsRrt = 1000;
            for(int iRrt  = 1; iRrt < maxIterationsRrt; iRrt++) {
                Node node(widthGenerator(rng),heightGenerator(rng),iRrt);
                ROS_INFO_STREAM("New node generated: " << node.id << " x: " << node.x << " y: " << node.y);
                
                Node nearestNode = node.FindNearestNode(Tree);
                Tree.push_back(nearestNode);
                ROS_INFO_STREAM("New node added to tree: " << nearestNode.id << " x: " << nearestNode.x << " y: " << nearestNode.y);
                
                xPathPos += stepLength;
                yPathPos += stepLength;
                
                pose.position.x = xPathPos;
                pose.position.y = yPathPos;
                pose.position.z = 0;

                pose.orientation.x = 0.924;
                pose.orientation.y = 0;
                pose.orientation.z = 0;
                pose.orientation.w = 0.383;

                poseStampedMsg.pose = pose;

                posesStampedVectorMsg.push_back(poseStampedMsg);
                
                distToGoal = calcDistance(xPathPos,yPathPos,xGoal,yGoal);
                /*if (distToGoal > goalDistThreshold) {
                    break;
                }*/
            }

            path.header.frame_id = frameIdMap;
            path.poses = posesStampedVectorMsg;
            return path;
        }

};


int main(int argc, char** argv) {

    ros::init(argc,argv, "global_planner");
    ros::NodeHandle n;
    
    boost::shared_ptr<geometry_msgs::PoseStamped const> iniPosMsg;
    iniPosMsg = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("slam_out_pose",ros::Duration(2.0));

    boost::shared_ptr<nav_msgs::MapMetaData const> mapMetaData;
    mapMetaData = ros::topic::waitForMessage<nav_msgs::MapMetaData>("map_metadata",ros::Duration(2.0));

    GlobalPlanner planner(iniPosMsg,mapMetaData);

    boost::shared_ptr<nav_msgs::OccupancyGrid const> mapData;
    mapData = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("map",ros::Duration(2.0));


    ros::Publisher pub = n.advertise<nav_msgs::Path>("global_path",10);

    ros::Rate r(10); // 10 hz

    nav_msgs::Path path = planner.createPath();

    while(ros::ok()) {

        pub.publish(path);

        ros::spinOnce();
        r.sleep();
    }

    
    return 0;

}