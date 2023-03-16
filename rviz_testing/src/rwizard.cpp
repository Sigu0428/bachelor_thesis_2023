
#include <cmath>
#include <vector>
#include "ros/ros.h"
#include "visualization_msgs/MarkerArray.h"
#include "discretization.h"
#include "sensor_msgs/PointCloud.h"

using namespace std;

DiscreteWorkspace w1(2,2,2.5,30);
//DiscreteWorkspace w1(2,0.05,2.5,40);

double threshold= 0.02;
bool received = 0;

void callbackfunc(const sensor_msgs::PointCloud::ConstPtr& msg){
    ROS_INFO("called back");
    vector<geometry_msgs::Point32> points=msg->points;

    for(int i=0;i<points.size();i++){
        if(msg->channels.at(0).values.at(i)<threshold){
            Point p(double(points.at(i).x),double(points.at(i).y),double(points.at(i).z),1);   
            w1.add_point(p);
            //ROS_INFO("Point(%f,%f,%f), val: %f",p.x,p.y,p.z,p.val);
        } 
    }

    received = 1;

}

int main( int argc, char** argv )
{
    //init ros, nodehandle and setup topic
    ros::init(argc, argv, "basic_shapes");
    ros::NodeHandle n;
    ros::Rate r(1);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);

    ros::Subscriber pointcloud_sub=n.subscribe<sensor_msgs::PointCloud>("workspacePointCloud",1,callbackfunc);
    
    
    //spin untill callback has been called
    ROS_INFO("WAITING FOR POINTCLOUD");
    while(!received){
        ros::spinOnce();
    };

    //apply brushfire and publish
    if(ros::ok()){
        ROS_INFO("Brushfire");
        w1.brushfire();
        ROS_INFO("pub");
        w1.publish_grid(marker_pub,"singularities"); //remember to change grid  
        //w1.publish_grid(marker_pub,"colorgradient");
    }

  
}

//TODO: replace point struct with geo msg