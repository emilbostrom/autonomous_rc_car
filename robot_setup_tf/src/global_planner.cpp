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
                    ROS_INFO_STREAM("New nearest node id: " << nearestNode.id);
                    ROS_INFO_STREAM("The distance to above is: " << nearestDist);
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
        int mapData[]; // [0-100] occupancy
        geometry_msgs::Pose mapOriginPose; 
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
            mapData = mapMsg->data;
            mapOriginPose = mapMsg->info.origin;

            ROS_INFO_STREAM("Map resolution [m]: " << mapResolution);
            ROS_INFO_STREAM("Map width [cells]: " << mapWidth);
            ROS_INFO_STREAM("Map height [cells]: " << mapHeight);
            
            uint32_t seed_val = 100;
            rng.seed(seed_val);
            widthGenerator = std::uniform_int_distribution<uint32_t>(0, mapWidth);
            heightGenerator = std::uniform_int_distribution<uint32_t>(0, mapHeight);

            stepLength = mapResolution*2;
            goalDistThreshold = stepLength*2;
            obstacleDistThreshold = stepLength*2;
        }

        std::tuple<double, double> CellToCoordinate(int cellWidth, int cellHeight) {
            double xPos = (cellWidth - mapWidth/2)*mapResolution;
            double yPos = (cellHeight - mapHeight/2)*mapResolution;
            return {xPos, yPos};
        }

        double calcDistance(double x1, double y1, double x2, double y2) {
            return sqrt(pow((x2-x1),2) + pow((y2-y1),2));
        }

        bool checkForObstacle(Node node) {
            for (int i = 0; i < mapData.size(); i++) {
                if (mapData[i] == 0){
                    continue;
                } 
                
                int xMapCell;
                int yMapCell  = i / mapHeight;
                if (i % mapWidth == 0) {
                    xMapCell = i;
                } 
                else {
                    xMapCell = i % mapWidth;
                }
                
                auto [xMapPos, yMapPos] = CellToCoordinate(xMapCell,yMapCell);
                double distToObstacle = calcDistance(xMapPos,yMapPos,node.xPos,node.yPos);
                
                if (distToObstacle < obstacleDistThreshold){
                    return true;
                }
            }
            return false;
        }

        nav_msgs::Path createPathToGoal(std::vector<Node> Tree) {
            
            std::vector<geometry_msgs::PoseStamped> posesStampedVectorMsg;
            
            geometry_msgs::PoseStamped poseStampedMsg;
            poseStampedMsg.header.frame_id = frameIdMap;
            
            geometry_msgs::Pose pose;
            
            nav_msgs::Path path;

            Node node = Tree.back();
            while(node.id != 0) {
                pose.position.x = node.xPos;
                pose.position.y = node.yPos;
                pose.position.z = 0;

                pose.orientation.x = 0.924;
                pose.orientation.y = 0;
                pose.orientation.z = 0;
                pose.orientation.w = 0.383;

                poseStampedMsg.pose = pose;

                posesStampedVectorMsg.push_back(poseStampedMsg);

                node = Tree[node.idParent];
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
            Node nodeOrigin(mapOriginPose.position.x,mapOriginPose.position.y,idOrigin,stepLength);
            nodeOrigin.idParent = 0;
            nodeOrigin.cost = 0;

            // Create an array of all nodes
            std::vector<Node> Tree;
            Tree.push_back(nodeOrigin);

            int maxIterationsRrt = 1000;
            for(int iRrt  = 1; iRrt < maxIterationsRrt; iRrt++) {
                
                int xCell = widthGenerator(rng);
                int yCell = heightGenerator(rng);

                auto [xPosNode, yPosNode] = CellToCoordinate(xCell,yCell);

                Node newNode(xPosNode,yPosNode,iRrt,stepLength);
                newNode.FindNearestNode(Tree);

                bool nodeInObstacle = checkForObstacle(newNode);
                if (nodeInObstacle) {
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