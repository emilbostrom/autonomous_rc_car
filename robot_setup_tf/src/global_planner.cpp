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
#include <algorithm> 
#include <stdlib.h>


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
        int id;
        int idParent;
        int cost;
        double stepLength;

        double xPos;
        double yPos;
        
        Node(int xPos, int yPos, int id, double stepLength) : 
             xPos(xPos), yPos(yPos), id(id), stepLength(stepLength){
            ROS_INFO_STREAM("Node id: " << this->id);
        }

        void calcNewNodePos(Node nearestNode, double distance) {
            double dx = static_cast<double>(xPos - nearestNode.xPos) / distance;
            double dy = static_cast<double>(yPos - nearestNode.yPos) / distance;
            this->xPos = nearestNode.xPos + dx * stepLength;
            this->yPos = nearestNode.yPos + dy * stepLength;
            ROS_INFO_STREAM("nearestNode.xPos: " << nearestNode.xPos);
            ROS_INFO_STREAM("dx * stepLength: " << dx * stepLength);
            ROS_INFO_STREAM("new xPos: " << this->xPos);
            ROS_INFO_STREAM("nearestNode.yPos: " << nearestNode.yPos);
            ROS_INFO_STREAM("dy * stepLength: " << dy * stepLength);
            ROS_INFO_STREAM("new yPos: " << this->yPos);
        }

        double calcDistance(double x1, double y1, double x2, double y2) {
            return std::max(sqrt(pow((x2-x1),2) + pow((y2-y1),2)),0.001);
        }

        void FindNearestNode(std::vector<Node> Tree) {
            Node nearestNode = Tree[0];

            ROS_INFO_STREAM("Node " << id << " has position " << xPos << "," << yPos);

            double nearestDist = calcDistance(xPos,yPos,Tree[0].xPos,Tree[0].yPos);

            ROS_INFO_STREAM("Tree size: " << Tree.size());

            for(int i = 1; i < Tree.size(); i++) {
                double dist = calcDistance(xPos,yPos,Tree[i].xPos,Tree[i].yPos);
                if (dist < nearestDist) {
                    nearestNode = Tree[i];
                    nearestDist = dist;
                }
            }

            ROS_INFO_STREAM("Nearest node to " << id << " is " << nearestNode.id 
                            << "with distance: " << nearestDist);

            this->idParent = nearestNode.id;

            calcNewNodePos(nearestNode,nearestDist);
        }

};


class GlobalPlanner{
    public:
        std::string frameIdMap = "map";
        double mapResolution; // [m/cell]
        int mapWidth; // [cells]
        int mapHeight; // [cells]
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
                      const nav_msgs::OccupancyGrid::ConstPtr& mapMsg) {
            
            ROS_INFO_STREAM("Received pose: " << poseMsg);
            xCurrent = poseMsg->pose.position.x;
            yCurrent = poseMsg->pose.position.y;
            zCurrent = poseMsg->pose.position.z;
            xQuat = poseMsg->pose.orientation.x;
            yQuat = poseMsg->pose.orientation.y;
            zQuat = poseMsg->pose.orientation.z;
            wQuat = poseMsg->pose.orientation.w;

            mapResolution = mapMsg->info.resolution;
            mapWidth = mapMsg->info.width;
            mapHeight = mapMsg->info.height;

            ROS_INFO_STREAM("Map resolution [m]: " << mapResolution);
            ROS_INFO_STREAM("Map width [cells]: " << mapWidth);
            ROS_INFO_STREAM("Map height [cells]: " << mapHeight);
            
            uint32_t seed_val = 100;
            rng.seed(seed_val);
            widthGenerator = std::uniform_int_distribution<uint32_t>(0, mapWidth);
            heightGenerator = std::uniform_int_distribution<uint32_t>(0, mapHeight);

            stepLength = mapResolution*2;
            goalDistThreshold = stepLength*2;
        }

        std::tuple<double, double> CellToCoordinate(int cellWidth, int cellHeight) {
            double xPos = (cellWidth - mapWidth/2)*mapResolution;
            double yPos = (cellHeight - mapHeight/2)*mapResolution;
            return {xPos, yPos};
        }

        double calcDistance(double x1, double y1, double x2, double y2) {
            return sqrt(pow((x2-x1),2) + pow((y2-y1),2));
        }

        const nav_msgs::Path createPath(ros::Publisher pub){
            
            nav_msgs::Path path;
            std::vector<geometry_msgs::PoseStamped> posesStampedVectorMsg;

            geometry_msgs::PoseStamped poseStampedMsg;
            poseStampedMsg.header.frame_id = frameIdMap;

            geometry_msgs::Pose pose;

            double xPathPos = xCurrent;
            double yPathPos = yCurrent;

            double distToGoal = calcDistance(xPathPos,yPathPos,xGoal,yGoal);


            // Create first node, which is current position
            Node nodeOrigin(0,0,0,stepLength);
            nodeOrigin.idParent = 0;
            nodeOrigin.cost = 0;

            // Create an array of all nodes
            std::vector<Node> Tree;
            Tree.push_back(nodeOrigin);

            int maxIterationsRrt = 1000;
            for(int iRrt  = 1; iRrt < maxIterationsRrt; iRrt++) {
                auto [xPosNode, yPosNode] = CellToCoordinate(widthGenerator(rng),heightGenerator(rng));
                Node newNode(xPosNode,yPosNode,iRrt,stepLength);
                newNode.FindNearestNode(Tree);
                Tree.push_back(newNode);

                ROS_INFO_STREAM("Added new node to tree: " << newNode.id << " xPos: " 
                                 << newNode.xPos << " yPos: " << newNode.yPos);
                
                pose.position.x = newNode.xPos;
                pose.position.y = newNode.yPos;
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
                pub.publish(path)
                ros::Duration(1).sleep();
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

    boost::shared_ptr<nav_msgs::OccupancyGrid const> mapData;
    mapData = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("map",ros::Duration(2.0));

    GlobalPlanner planner(iniPosMsg,mapData);

    ros::Publisher pub = n.advertise<nav_msgs::Path>("global_path",10);

    ros::Rate r(10); // 10 hz

    nav_msgs::Path path = planner.createPath(pub);

    while(ros::ok()) {

        pub.publish(path);

        ros::spinOnce();
        r.sleep();
    }

    
    return 0;

}