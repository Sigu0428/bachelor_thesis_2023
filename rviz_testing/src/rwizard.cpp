
#include <cmath>
#include <vector>
#include "ros/ros.h"
#include "visualization_msgs/MarkerArray.h"
#include "discretization.h"
#include "sensor_msgs/PointCloud.h"

using namespace std;

//---------------------------------------------------PARAMETERS------------------------------------------------------------------------------------

double threshold= 0.08;

int max_sniffs=1;
int N_points=100000;

#define GRADIENTSLICE 10
//"manipslice"; //"gradientslice" "singularvoxels" value: res

string launchpath="roslaunch ~/catkin_ws/src/Afrovenator/mycode/src/genPointCloud.launch num_of_points:="+to_string(N_points);
//-------------------------------------------------------------------------------------------------------------------------------------------------

bool received = 0;

#ifdef MANIPSLICE
    DiscreteWorkspace w1(2,0.05,2.5,MANIPSLICE);
#endif
#ifdef GRADIENTSLICE
    DiscreteWorkspace w1(2,0.05,2.5,GRADIENTSLICE);
#endif
#ifdef SINGULARVOXELS
    DiscreteWorkspace w1(2,2,2.5,SINGULARVOXELS);
#endif


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


void add_average_cb(const sensor_msgs::PointCloud::ConstPtr& msg){ //add points to containergrid
    ROS_INFO("called back");
    vector<geometry_msgs::Point32> points=msg->points;

    ROS_INFO("Cloudsize: %d",msg->points.size());
    for(int i=0;i<points.size();i++){
        Point p(double(points.at(i).x),double(points.at(i).y),double(points.at(i).z),msg->channels.at(0).values.at(i));   
        w1.add_point_to_container(p);
    }
    received = 1;

}


int main( int argc, char** argv )
{
    //init ros, nodehandle and setup topic
    ros::init(argc, argv, "basic_shapes");
    ros::NodeHandle n;
    ros::Rate r(10);
    ros::Rate r_long(0.2);

    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    ros::Publisher marker_array_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
    ros::Subscriber pointcloud_sub=n.subscribe<sensor_msgs::PointCloud>("panda_link0_sc/workspacePointCloud",1,add_average_cb); //call cb function when new message is on topic
    r.sleep();


    //spin untill callback has been called
    ROS_INFO("WAITING FOR POINTCLOUD");
    int sniffs=0;
    while(max_sniffs-sniffs){ 
        
        system(launchpath.c_str());
        while(!received){ //Spin and sleep until pointcloud is published to topic
            ros::spinOnce(); 
            r.sleep();
        };
        received=0;
        sniffs++;
        ROS_INFO("sniffed %d time(s)",sniffs);
    }
    

    //generate _grid from _containergrid averages and publish
    if(ros::ok()){
        ROS_INFO("pub");
        #ifdef MANIPSLICE
            w1.generate_average_grid();
            w1.average_to_manipulability_grid();
            w1.publish_grid(marker_array_pub,"manipulabilitygradient");
        #endif
        #ifdef GRADIENTSLICE
            w1.generate_average_grid();
            w1.average_to_singular_grid(threshold);

            w1.publish_grid(marker_array_pub,"singularitygradient");
            w1.sphere_fit_singularities();
            w1.publish_sphere_fit(marker_pub);
            r_long.sleep();
            w1.add_singularities_to_fit();
            r_long.sleep();
            w1.publish_grid(marker_array_pub,"singularitygradient");
            w1.fill_unreachable_areas();
            w1.brushfire();
            r_long.sleep();
            w1.publish_grid(marker_array_pub,"singularitygradient");
        #endif
        #ifdef SINGULARVOXELS
            w1.generate_average_grid();
            w1.average_to_singular_grid(threshold);
            w1.publish_grid(marker_array_pub,"singularities");
            w1.sphere_fit_singularities();
            w1.publish_sphere_fit(marker_pub);
            /*
            r_long.sleep();
            w1.add_singularities_to_fit();

            w1.publish_grid(marker_array_pub,"singularities");
            */
        #endif
          
    }

  
}

//TODO: replace point struct with geo msg

    //w1.average_to_singular_grid(threshold);
    //ROS_INFO("Brushfire");
    //w1.brushfire();