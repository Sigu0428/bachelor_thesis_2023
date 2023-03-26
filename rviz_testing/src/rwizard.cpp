
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

#define COMBINEDGRADIENTSLICE 15
//"manipslice"; //"gradientslice" "singularvoxels" value: res

string launchpath="roslaunch ~/catkin_ws/src/Afrovenator/mycode/src/genPointCloud.launch num_of_points:="+to_string(N_points);
//-------------------------------------------------------------------------------------------------------------------------------------------------

int received = 0;

#ifdef MANIPSLICE
    DiscreteWorkspace w1(2,0.05,2.5,MANIPSLICE);
    DiscreteWorkspace w2(3,0.05,2.5,MANIPSLICE);
#endif
#ifdef GRADIENTSLICE
    DiscreteWorkspace w1(2,0.05,2.5,GRADIENTSLICE);
    DiscreteWorkspace w2(3,0.05,2.5,GRADIENTSLICE);
#endif
#ifdef SINGULARVOXELS
    DiscreteWorkspace w1(2,2,2.5,SINGULARVOXELS);
    DiscreteWorkspace w2(3,0.05,2.5,SINGULARVOXELS);
#endif
#ifdef COMBINEDGRADIENTSLICE
    DiscreteWorkspace w1(3,0.05,2.5,COMBINEDGRADIENTSLICE);
    DiscreteWorkspace w2(3,0.05,2.5,COMBINEDGRADIENTSLICE);
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


void pointcloud_to_average1(const sensor_msgs::PointCloud::ConstPtr& msg){ //add points to containergrid
    ROS_INFO("called back, header frame: %s",msg->header.frame_id.c_str());

    ROS_INFO("ID %d",msg->header.seq);
    vector<geometry_msgs::Point32> points=msg->points;
    ROS_INFO("Cloudsize: %d",msg->points.size());

    for(int i=0;i<points.size();i++){
        Point p(double(points.at(i).x),double(points.at(i).y),double(points.at(i).z),msg->channels.at(0).values.at(i));   
        w1.add_point_to_container(p);
    }
    received++;
}


void pointcloud_to_average2(const sensor_msgs::PointCloud::ConstPtr& msg){ //add points to containergrid
    ROS_INFO("called back, header frame: %s",msg->header.frame_id.c_str());

    ROS_INFO("ID %d",msg->header.seq);
    vector<geometry_msgs::Point32> points=msg->points;
    ROS_INFO("Cloudsize: %d",msg->points.size());

    for(int i=0;i<points.size();i++){
        Point p(double(points.at(i).x),double(points.at(i).y),double(points.at(i).z),msg->channels.at(0).values.at(i));   
        w2.add_point_to_container(p);
    }
    received++;
}

//issues: cannot associate cloud with workspace based on message content, so we require a cb func per robot

int main( int argc, char** argv )
{
    //init ros, nodehandle and setup topic
    ros::init(argc, argv, "basic_shapes");
    ros::NodeHandle n;
    ros::Rate r(10);
    ros::Rate r_long(0.4);

    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    ros::Publisher marker_array_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);

    ros::Subscriber PCsub1=n.subscribe<sensor_msgs::PointCloud>("robot1/workspacePointCloud",1,pointcloud_to_average1); //call cb function when new message is on topic
    ros::Subscriber PCsub2=n.subscribe<sensor_msgs::PointCloud>("robot2/workspacePointCloud",1,pointcloud_to_average2); //call cb function when new message is on topic

    r.sleep();


    ROS_INFO("WAITING FOR POINTCLOUD");
    while(received<2){ //Spin and sleep until pointcloud is published to topic
        ros::spinOnce(); 
        r.sleep();
    };
    //spin untill callback has been called
    
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
            
            r_long.sleep();
            w1.publish_grid(marker_array_pub,"singularitygradient");
            
            
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
        #ifdef COMBINEDGRADIENTSLICE
            w1.generate_average_grid();
            w1.average_to_singular_grid(threshold);
                
            w1.publish_grid(marker_array_pub,"singularitygradient");
            
            w1.sphere_fit_singularities();
            r_long.sleep();
            w1.publish_sphere_fit(marker_pub);
            r_long.sleep();
            w1.add_singularities_to_fit();
            r_long.sleep();
            w1.publish_grid(marker_array_pub,"singularitygradient");
            w1.fill_unreachable_areas();
            r_long.sleep();
            w1.publish_grid(marker_array_pub,"singularitygradient");

            r_long.sleep();

            w2.generate_average_grid();
            w2.average_to_singular_grid(threshold);

            w2.publish_grid(marker_array_pub,"singularitygradient");
            w2.sphere_fit_singularities();

            r_long.sleep();
            w2.publish_sphere_fit(marker_pub);
            r_long.sleep();
            w2.add_singularities_to_fit();
            r_long.sleep();
            w2.publish_grid(marker_array_pub,"singularitygradient");
            w2.fill_unreachable_areas();
            r_long.sleep();
            w2.publish_grid(marker_array_pub,"singularitygradient");

            r_long.sleep();
            
            DiscreteWorkspace w3(w1,w2,COMBINEDGRADIENTSLICE);
            w3.publish_grid(marker_array_pub,"singularitygradient");
            
            w3.brushfire();
            r_long.sleep();
            w3.publish_grid(marker_array_pub,"singularitygradient");
            

        #endif
          
    }

  
}
/* old multicloud sniffer
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
    

*/