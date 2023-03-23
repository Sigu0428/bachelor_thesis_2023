
#include <cmath>
#include <vector>
#include "ros/ros.h"
#include "visualization_msgs/MarkerArray.h"
#include "discretization.h"
#include "sensor_msgs/PointCloud.h"

using namespace std;

//---------------------------------------------------PARAMETERS------------------------------------------------------------------------------------
//DiscreteWorkspace w1(2,2,2.5,30);
DiscreteWorkspace w1(2,0.05,2.5,50);

double threshold= 0.02;
bool received = 0;

int max_sniffs=10;

//-------------------------------------------------------------------------------------------------------------------------------------------------



void add_singularities_cb(const sensor_msgs::PointCloud::ConstPtr& msg){
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


void add_average_cb(const sensor_msgs::PointCloud::ConstPtr& msg){
    ROS_INFO("called back");
    vector<geometry_msgs::Point32> points=msg->points;


    ROS_INFO("Cloudsize: %d",msg->points.size());
    for(int i=0;i<points.size();i++){
        
        Point p(double(points.at(i).x),double(points.at(i).y),double(points.at(i).z),msg->channels.at(0).values.at(i));   
        w1.add_point_to_container(p);
        
    }
    //average and add singularities to grid
    //w1.average_to_singular_grid(threshold);
    //ROS_INFO("Brushfire");
    //w1.brushfire();
    

    received = 1;

}

int main( int argc, char** argv )
{
    //init ros, nodehandle and setup topic
    ros::init(argc, argv, "basic_shapes");
    ros::NodeHandle n;
    ros::Rate r(10);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);

    ros::Subscriber pointcloud_sub=n.subscribe<sensor_msgs::PointCloud>("workspacePointCloud",1,add_average_cb);
    

    //spin untill callback has been called
    ROS_INFO("WAITING FOR POINTCLOUD");
    int sniffs=0;
    while(max_sniffs-sniffs){
        while(!received){
            ros::spinOnce();
            r.sleep();
        };
        received=0;
        sniffs++;
        ROS_INFO("sniffed %d time(s)",sniffs);

    }
    
    //modify grid and publish
    if(ros::ok()){
        ROS_INFO("pub");
        //w1.publish_grid(marker_pub,"singularities"); //remember to change grid  
          
        w1.average_to_manipulability_grid();
        w1.publish_grid(marker_pub,"colorgradient");
    }

  
}

//TODO: replace point struct with geo msg