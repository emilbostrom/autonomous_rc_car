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


double xGoal = 0.5;
double yGoal = 3;
double z_goal = 0;
double x_quat_goal = 0.707;
double y_quat_goal = 0;
double z_quat_goal = 0;
double w_quat_goal = 0.707;



std::vector<geometry_msgs::PoseStamped::ConstPtr> pose;

class Node{
    public:
        int idParent;
        int cost;

        Node(double xPos, double yPos, int id, double stepLength) : 
             xPos(xPos), yPos(yPos), id(id), stepLength(stepLength){
            ROS_INFO_STREAM("Node id: " << this->id);
        }

        void calcNewNodePos(Node nearestNode, double distance) {
            ROS_INFO_STREAM("Distance to node for connection: " << distance);
            double dx = static_cast<double>(xPos - nearestNode.xPos) / distance;
            double dy = static_cast<double>(yPos - nearestNode.yPos) / distance;
            this->xPos = nearestNode.xPos + dx * stepLength;
            this->yPos = nearestNode.yPos + dy * stepLength;
            ROS_INFO_STREAM("new xPos: " << this->xPos);
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
                    //ROS_INFO_STREAM("New nearest node id: " << nearestNode.id);
                    //ROS_INFO_STREAM("The distance to above is: " << nearestDist);
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
        //std::vector<int8_t> mapDataFetch; // [0-100] occupancy
        std::vector<int> mapData;
        double stepLength; // [m]
        double goalDistThreshold; // [m]
        double obstacleDistThreshold; // [m]


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

        std::uniform_int_distribution<uint32_t> nodeIsGoalBias = std::uniform_int_distribution<uint32_t>(0, 99);
        int goalBias = 10; // Tuning parameter

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
            for (const auto& value : mapMsg->data) {
                mapData.push_back(static_cast<int>(value));
            }

            /*for(int i = 0; i < mapData.size();i++){
                if (mapData[i] != -1) {
                    ROS_INFO_STREAM("index: "<< i);
                    ROS_INFO_STREAM("value: "<< mapData[i]);
                    ROS_INFO_STREAM("x: " << i % mapHeight);
                    ROS_INFO_STREAM("y: " << i / mapHeight);
                }
            }*/

            ROS_INFO_STREAM("Map resolution [m]: " << mapResolution);
            ROS_INFO_STREAM("Map width [cells]: " << mapWidth);
            ROS_INFO_STREAM("Map height [cells]: " << mapHeight);
            ROS_INFO_STREAM("Map data size: " << mapData.size());
            ROS_INFO_STREAM("Map data first value: " << mapData[0]);
            
            uint32_t seed_val = 100;
            rng.seed(seed_val);
            widthGenerator = std::uniform_int_distribution<uint32_t>(0, mapWidth);
            heightGenerator = std::uniform_int_distribution<uint32_t>(0, mapHeight);

            stepLength = mapResolution*2;
            goalDistThreshold = stepLength*2;
            obstacleDistThreshold = stepLength*2;
        }

        std::tuple<double, double> CellToCoordinate(int xCell, int yCell) {
            double xPos = (xCell - mapWidth/2)*mapResolution;
            double yPos = (yCell - mapHeight/2)*mapResolution;
            return {xPos, yPos};
        }

        double calcDistance(double x1, double y1, double x2, double y2) {
            return sqrt(pow((x2-x1),2) + pow((y2-y1),2));
        }

        bool checkForObstacle(Node node) {
            int xMapCell = node.xPos / mapResolution + mapWidth/2;
            int yMapCell = node.yPos / mapResolution + mapHeight/2;

            int mapDataIndex = yMapCell*mapHeight + xMapCell;
            
            //ROS_INFO_STREAM("Map data index: " << mapDataIndex);
            //ROS_INFO_STREAM("Node position " << node.xPos << "," << node.yPos <<  " is in mapdata: " << this->mapData[mapDataIndex]);
            if (this->mapData[mapDataIndex] != 0){
                return true;
            } else {
                return false;
            }
        }

        nav_msgs::Path createPathToGoal(std::vector<Node> Tree) {
            
            ROS_INFO_STREAM("Goal node reached, creating path msg");

            std::vector<geometry_msgs::PoseStamped> posesStampedVectorMsg;
            
            geometry_msgs::PoseStamped poseStampedMsg;
            poseStampedMsg.header.frame_id = frameIdMap;
            
            geometry_msgs::Pose pose;
            
            nav_msgs::Path path;

            Node nodePrev = Tree.back();
            ROS_INFO_STREAM("First node id: " << nodePrev.id);
            ROS_INFO_STREAM("First node parent id: " << nodePrev.idParent);
            
            for (int i = Tree.size() - 1; i >= 0; --i) {
                Node node = Tree[i];
                ROS_INFO_STREAM("Node id: " << node.id);
                if(node.id != nodePrev.idParent) {
                    continue;
                }
                pose.position.x = node.xPos;
                pose.position.y = node.yPos;
                pose.position.z = 0;

                pose.orientation.x = 0.924;
                pose.orientation.y = 0;
                pose.orientation.z = 0;
                pose.orientation.w = 0.383;

                poseStampedMsg.pose = pose;

                posesStampedVectorMsg.push_back(poseStampedMsg);

                nodePrev = node;
            }

            path.header.frame_id = frameIdMap;
            path.poses = posesStampedVectorMsg;

            return path;
        }

        const nav_msgs::Path createPath(ros::Publisher pub){

            double xPathPos = xCurrent;
            double yPathPos = yCurrent;
            nav_msgs::Path path;

            double distToGoal = calcDistance(xPathPos,yPathPos,xGoal,yGoal);

            // Create first node, which is current position
            int idOrigin = 0;
            Node nodeOrigin(xCurrent,yCurrent,idOrigin,stepLength);
            nodeOrigin.idParent = 0;
            nodeOrigin.cost = 0;

            // Create an array of all nodes
            std::vector<Node> Tree;
            Tree.push_back(nodeOrigin);

            int maxIterationsRrt = 20000;
            for(int iRrt  = 1; iRrt < maxIterationsRrt; iRrt++) {
                
                int xCell = widthGenerator(rng);
                int yCell = heightGenerator(rng);

                auto [xPosNode, yPosNode] = CellToCoordinate(xCell,yCell);

                // 
                if(nodeIsGoalBias(rng) < goalBias){
                    ROS_INFO_STREAM("Node is set to goal");
                    xPosNode = xGoal;
                    yPosNode = yGoal;
                }

                Node newNode(xPosNode,yPosNode,iRrt,stepLength);
                newNode.FindNearestNode(Tree);

                bool nodeInObstacle = checkForObstacle(newNode);
                if (nodeInObstacle) {
                    ROS_INFO_STREAM("Node in obstacle, skipped");
                    continue;
                }
 
                Tree.push_back(newNode);

                ROS_INFO_STREAM("Added new node to tree: " << newNode.id << " xPos: " 
                                 << newNode.xPos << " yPos: " << newNode.yPos);
                
                distToGoal = calcDistance(newNode.xPos,newNode.yPos,xGoal,yGoal);
                if (distToGoal < goalDistThreshold) {
                    path = createPathToGoal(Tree);
                    return path;
                }
            }
            path = createPathToGoal(Tree);
            return path;
        }

};


int main(int argc, char** argv) {

    ros::init(argc,argv, "global_planner");
    ros::NodeHandle n;
    
    boost::shared_ptr<geometry_msgs::PoseStamped const> iniPosMsg;
    iniPosMsg = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("slam_out_pose",ros::Duration(2.0));

    boost::shared_ptr<nav_msgs::OccupancyGrid const> mapDataMsg;
    mapDataMsg = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("map",ros::Duration(2.0));

    GlobalPlanner planner(iniPosMsg,mapDataMsg);

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