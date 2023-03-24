#ifndef H_DISCRETIZATION
#define H_DISCRETIZATION

#include <vector>
#include "ros/ros.h"
#include "discretization.h"
#include <cmath>
#include "visualization_msgs/MarkerArray.h"

using namespace std;

struct Point{ //point in space
    Point(double x_in,double y_in, double z_in, double val_in){
        x=x_in;
        y=y_in;
        z=z_in;
        val=val_in;
    }
    double x;
    double y;  
    double z;
    double val;
};

struct Coordinate{  //grid index
    Coordinate(double x_in,double y_in, double z_in){
        x=x_in;
        y=y_in;
        z=z_in;
    }
    int x;
    int y;
    int z;
};


class DiscreteWorkspace{

public:
    DiscreteWorkspace(double,double,double,int); //0 for empty voxels

    double & operator()(int,int,int);
    double & operator()(Coordinate);

    bool add_point(Point); //adds point to grid and _points
    bool add_point_to_container(Point); //adds point to _containergrid and _points

    void generate_average_grid(); //fill grid cells with average from _containergrid, cells with no samples average to 0

    void average_to_singular_grid(double); //use averaged manipulability to define singularities (1) based on threshold
    void average_to_manipulability_grid(); //use scale averaged manipulability grid for visualization

    void brushfire(); //generate brushfire on grid based on 

    void rebuild_grids(double); //rescale and rebuild grid and container from _points

    //conversions
    Coordinate point_to_coordinate(Point);
    Point coordinate_to_point(Coordinate); //places point and center of voxel

    void publish_grid(ros::Publisher&,string);
    
    ~DiscreteWorkspace();


private:

    //Physical workspace size
    double _x_size;     //[m]
    double _y_size;
    double _z_size;        

    int _resolution; //[voxels/m]
    //Discretized workspace size
    int _x_voxels;
    int _y_voxels;
    int _z_voxels;

    //Trackers for min and max brushfire/manipulability
    double _maxfire=1; //manximum cell value from brushfire
    double _max_manip=0; //maximum manipulability for a cell
    double _min_manip=1; //minimum non-zero manipulability for a cell

    //data 
    vector<vector<vector<double>>> _grid;
    vector<vector<vector<vector<double>>>> _containergrid;

    vector<Point> _points; //list of added samples, ONLY USED TO RESCALE GRIDS
    vector<Point> _singularities; //list of singularities, used to merge grids

    //methods
    void build_grid();
    void build_container(); //build _containergrid with a 0 in each voxel
    bool in_bounds(int,int,int);
    bool in_bounds(Coordinate);
    bool add_point_to_container_only(Point); //add point to container, used by rebuild (we dont want duplicates in _points)
    bool add_point_to_grid_only(Point); //add point to grid, used by rebuild (we dont want duplicates in _points)

};


DiscreteWorkspace::DiscreteWorkspace(double x,double y,double z, int res)
{
    _x_size=x;
    _y_size=y;
    _z_size=z;
    _resolution=res;
    build_grid();
    build_container();
}

DiscreteWorkspace::~DiscreteWorkspace(){};

void DiscreteWorkspace::build_grid(){
    _x_voxels=ceil(_x_size*_resolution);
    _y_voxels=ceil(_y_size*_resolution);
    _z_voxels=ceil(_z_size*_resolution);

    vector<double> z_cells(_z_voxels,0);
    vector<vector<double>> y_cells(_y_voxels,z_cells);
    vector<vector<vector<double>>> x_cells(_x_voxels,y_cells);
    _grid=x_cells;
}

void DiscreteWorkspace::build_container(){
    vector<double> list(1,0); //initialize every cell with a 0 to ensure all averages are defined
    vector<vector<double>> z_cells(_z_voxels,list);
    vector<vector<vector<double>>> y_cells(_y_voxels,z_cells);
    vector<vector<vector<vector<double>>>> x_cells(_x_voxels,y_cells);
    _containergrid=x_cells;
}

bool DiscreteWorkspace::in_bounds(int x,int y,int z){
    if(((((x<_x_voxels)&(y<_y_voxels)&(z<_z_voxels)))&((x>=0)&(y>=0)&(z>=0)))){
        return 1;
    } else {
        return 0;
    }
}

bool DiscreteWorkspace::in_bounds(Coordinate c){
    return in_bounds(c.x,c.y,c.z);
}

double & DiscreteWorkspace::operator()(int x,int y,int z){
    return _grid.at(x).at(y).at(z); 
}

double & DiscreteWorkspace::operator()(Coordinate c){
    return _grid.at(c.x).at(c.y).at(c.z); 
}

Coordinate DiscreteWorkspace::point_to_coordinate(Point p){
    Coordinate c(round(p.x*_resolution)+round(0.5*_x_voxels),round(p.y*_resolution)+round(0.5*_y_voxels),round(p.z*_resolution)+round(0.5*_z_voxels));
    return c;
}

Point DiscreteWorkspace::coordinate_to_point(Coordinate c){
    Point p((c.x-round(0.5*_x_voxels))/_resolution,(c.y-round(0.5*_y_voxels))/_resolution,(c.z-round(0.5*_z_voxels))/_resolution,(*this)(c));
    return p;
}

bool DiscreteWorkspace::add_point_to_grid_only(Point p){
    //assumes points are in [m]
    if(in_bounds(point_to_coordinate(p))){
        (*this)(point_to_coordinate(p))=p.val;   
        return 1;    
    } else {
        return 0;
    }

}

bool DiscreteWorkspace::add_point_to_container_only(Point p){
    //assumes points are in [m]
    Coordinate c=point_to_coordinate(p);
    if(in_bounds(c)){
            _containergrid.at(c.x).at(c.y).at(c.z).push_back(p.val);
        return 1;    
    } else {
        return 0;
    }

}

bool DiscreteWorkspace::add_point(Point p){
    //assumes points are in [m]
    if(add_point_to_grid_only(p)){
        _points.push_back(p); //add point to _points
        return 1;    
    } else {
        return 0;
    }

}

bool DiscreteWorkspace::add_point_to_container(Point p){
    //assumes points are in [m]
    Coordinate c=point_to_coordinate(p);
    if(in_bounds(c)){
            _containergrid.at(c.x).at(c.y).at(c.z).push_back(p.val);
            _points.push_back(p);
        return 1;    
    } else {
        return 0;
    }
}

void DiscreteWorkspace::generate_average_grid(){
    for(int x=0;x<_x_voxels;x++){
        for(int y=0;y<_y_voxels;y++){
            for(int z=0;z<_z_voxels;z++){
                double sum=0;
                for(int i=0;i<_containergrid.at(x).at(y).at(z).size();i++){
                    sum+=_containergrid.at(x).at(y).at(z).at(i);
                }
                double avg=sum/_containergrid.at(x).at(y).at(z).size();
                (*this)(x,y,z)=avg;
                if(avg>_max_manip){
                    _max_manip=avg;
                }
                if((avg<_min_manip)&&(avg!=0)){
                    _min_manip=avg;
                }
                //add avg point to _points
            }
        }
    }
}

void DiscreteWorkspace::average_to_singular_grid(double threshold){
    for(int x=0;x<_x_voxels;x++){
        for(int y=0;y<_y_voxels;y++){
            for(int z=0;z<_z_voxels;z++){
                if(((*this)(x,y,z)<threshold)&&((*this)(x,y,z)!=0)){ //if manipulability is 0 after averaging, there is no sample, and we ignore voxel
                    (*this)(x,y,z)=1; 
                    _singularities.push_back(coordinate_to_point(Coordinate(x,y,z)));
                } else {
                    (*this)(x,y,z)=0;
                }
            }
        }
    }
}

void DiscreteWorkspace::average_to_manipulability_grid(){
    ROS_INFO("min %f",_min_manip);
    //normalize grid values
    double factor=1./_min_manip;
    for(int x=0;x<_x_voxels;x++){
        for(int y=0;y<_y_voxels;y++){
            for(int z=0;z<_z_voxels;z++){
                double old_val=(*this)(x,y,z);
                double new_val=old_val*factor;
                (*this)(x,y,z)=new_val;
                ROS_INFO("old value: %f, factor: %f ,scaled value: %f",old_val,factor,new_val);
            }
        }
    }
    _max_manip=_max_manip*factor;
}

void DiscreteWorkspace::rebuild_grids(double resolution){
    _resolution=resolution;
    build_grid();
    build_container();
    for(int i=0;i<_points.size();i++){
        add_point_to_grid_only(_points.at(i));
        add_point_to_container_only(_points.at(i));
    }
}

void DiscreteWorkspace::brushfire(){
    vector<Coordinate> newly_updated;
    //add 1 value voxels to newly_updated
    for(int x=0;x<_x_voxels;x++){
        for(int y=0;y<_y_voxels;y++){
            for(int z=0;z<_z_voxels;z++){
                if(((*this)(x,y,z))==1){
                    Coordinate c(x,y,z);
                    newly_updated.push_back(c);
                }
            }
        }
    }
    ROS_INFO("1s: %d",newly_updated.size());
    int value=2;
    while(newly_updated.size()>0){ //while new vals added
        vector<Coordinate> prev_updated=newly_updated; //set to prev vals
        newly_updated.clear();
        for(int i=0;i<prev_updated.size();i++){ //iterate prev vals
            //check nbs
            for(int x_offset=-1;x_offset<=1;x_offset++){
                for(int y_offset=-1;y_offset<=1;y_offset++){
                    for(int z_offset=-1;z_offset<=1;z_offset++){
                        Coordinate nb(prev_updated.at(i).x+x_offset,prev_updated.at(i).y+y_offset,prev_updated.at(i).z+z_offset); 
                        if(in_bounds(nb)){ //check nbs of prev val
                            if(((*this)(nb))==0){ //if in bounds and ==0, apply current brushfire value
                                (*this)(nb)=value;
                                newly_updated.push_back(nb);
                            }
                        }
                    }
                }
            }
        }
        _maxfire=value;
        value++;
    }
}

void DiscreteWorkspace::publish_grid(ros::Publisher &pub, string setting="singularities"){
  //MARKER SETUP-------------------------------------------------------------------------------------------------------------------------------------------------------------
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "panda_link0_sc"; //SET TO PANDA LINK 0
    marker.header.stamp = ros::Time::now();


    // Set the marker type. 
    marker.type =  visualization_msgs::Marker::CUBE;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 1.0/_resolution;
    marker.scale.y = 1.0/_resolution;
    marker.scale.z = 1.0/_resolution;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.b = 0.0f;
    marker.color.g = 1.0f;

    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();

    //Create marker array with markers based on setting
    int m_id=0;

    visualization_msgs::MarkerArray marker_array;
    for(int x=0;x<_x_voxels;x++){
        for(int y=0;y<_y_voxels;y++){
            for(int z=0;z<_z_voxels;z++){
                // Set the namespace and id for this marker.  This serves to create a unique ID
                // Any marker sent with the same namespace and id will overwrite the old one
                marker.id=m_id++;
                marker.ns="voxel"+to_string(marker.id);

                //(1.0-(((*this)(x,y,z))/_maxfire));                     
                //ROS_INFO("value: %f maxfire: %f frac: %f",(*this)(x,y,z),_maxfire,marker.color.a);

                marker.pose.position.x = coordinate_to_point(Coordinate(x,y,z)).x+1.0/(2*_resolution);
                marker.pose.position.y = coordinate_to_point(Coordinate(x,y,z)).y+1.0/(2*_resolution);
                marker.pose.position.z = coordinate_to_point(Coordinate(x,y,z)).z+1.0/(2*_resolution);


                marker.color.r=0;

                //ROS_INFO("x: %d, y: %d, z: %d, val: %d, alpha: %f",x,y,z,(*this)(x,y,z),marker.color.a);
                //threshold code
                //VISUALIZATION SETTINGS
                if(setting=="singularities"){
                    if(((*this)(x,y,z))==1){
                        marker_array.markers.push_back(marker); //MarkerArray only member is vector<visualization_msgs::Marker> markers ^_^
                    }
                } else if(setting=="manipulabilitygradient"){
                    marker.color.b=min(1*((*this)(x,y,z))/_max_manip,1.0); //max is taken so that gradient works regardless of if brushfire or manipubility grid is being visualized
                    marker.color.g=max(1.0-1*((*this)(x,y,z))/_max_manip,0.0);
                    marker_array.markers.push_back(marker);
                    if((*this)(x,y,z)==0){
                        marker.color.b=0;
                        marker.color.g=0;
                        marker.color.r=1;
                    }
                } else if(setting=="singularitygradient"){
                    marker.color.b=min(3*((*this)(x,y,z))/_maxfire,1.0); //max is taken so that gradient works regardless of if brushfire or manipubility grid is being visualized
                    marker.color.g=max(1.0-3*((*this)(x,y,z))/_maxfire,0.0);
                    marker_array.markers.push_back(marker);
                    if((*this)(x,y,z)==0){
                        marker.color.b=0;
                        marker.color.g=0;
                        marker.color.r=1;
                    }

                }


            }
        }
    }


    //MARKER SETUP-------------------------------------------------------------------------------------------------------------------------------------------------------------

    ROS_INFO("Voxels: %d",marker_array.markers.size());

    //spin untill ros is ok, publish array if subscriber is there
    while(true){
        if(ros::ok()){
            if(pub.getNumSubscribers() > 0){
               ROS_INFO("SUBSCRIBER DETECteD");
                pub.publish(marker_array);
                sleep(1);
                break;
            } else {
                ROS_WARN_ONCE("Please create a subscriber to the marker");
                sleep(1);
            }
        } else {
            exit(404); //die if ros is not okay
        }
    }

}


#endif