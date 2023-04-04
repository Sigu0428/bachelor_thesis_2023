#ifndef H_DISCRETIZATION
#define H_DISCRETIZATION

#include <vector>
#include "ros/ros.h"
#include "discretization.h"
#include <cmath>
#include "visualization_msgs/MarkerArray.h"
#include "quickhull/QuickHull.hpp"

//CHECK LICENSE
#include "nr3.h"
#include "svd.h"
#include "utilities.h"



using namespace std;

struct Point{ //point in space
    Point(double x_in,double y_in, double z_in, double val_in){
        x=x_in;
        y=y_in;
        z=z_in;
        val=val_in;
    }
    bool operator==(Point p){
        if((x=p.x)&&(y=p.y)&&(z=p.z)){
            return 1;
        } else {
            return 0;
        }
    }

    double x;
    double y;  
    double z;
    double val;
};

struct Vector{
    Vector(){
        x=0; y=0; z=0;
    }
    Vector(double x_in,double y_in,double z_in){
        x=x_in;
        y=y_in;
        z=z_in;
    }
    Vector(Point p2, Point p1){
        x=p2.x-p1.x;
        y=p2.y-p1.y;
        z=p2.z-p1.z;
    }
    double size(){
        sqrt(pow(x,2)+pow(y,2)+pow(z,2));
    }
    Vector operator+(Vector b){
        return(Vector(x+b.x,y+b.y,z+b.z));
    }
    Vector operator*(double b){
        return(Vector(x*b,y*b,z*b));
    }
    void operator=(Vector b){
        x=b.x; y=b.y; z=b.z;
    }
    Point operator+(Point p){
        return(Point(x+p.x,y+p.y,z+p.z,0));
    }
    void normalize(){
        x=x/size();
        y=y/size();
        z=z/size();
    }
    double x;
    double y;  
    double z;
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

struct Sphere{
    Sphere(double a_in,double b_in, double c_in, double r_in){
        a=a_in;
        b=b_in;
        c=c_in;
        r=r_in;
    }
    Sphere(){};
    double a;
    double b;  
    double c;
    double r;
};

class DiscreteWorkspace{

public:
    DiscreteWorkspace(double,double,double,int); //0 for empty voxels

    DiscreteWorkspace(DiscreteWorkspace&,DiscreteWorkspace&,int); //assumes workspaces passed with _grid containing empty spaces=0 and singularities=1

    double & operator()(int,int,int);
    double & operator()(Coordinate);

    //bool add_point(Point); //adds point to grid and _points //REMOVE, WE ONLY WANT TO TOUCH _GRID WITH BRUSHFIRE/SINGULAR GRID
    bool add_point_to_container(Point); //adds point to _containergrid and _points

    void generate_average_grid(); //fill grid cells with average from _containergrid, cells with no samples average to 0

    void average_to_singular_grid(double); //use averaged manipulability to define singularities (1) in _grid based on threshold
    void average_to_manipulability_grid(); //use scale averaged manipulability grid for visualization

    void brushfire(); //generate brushfire on grid based on singularities

    void fit_sphere_to_singularities(); //fits sphere to _singularities
    void add_singularities_from_fit(); //adds singularities at intersection between fitted sphere and grid to _grid (doesnt append to _singularities)
    void fill_unreachable_areas(); //fills outside of workspace with singularities

    double grid_volume(); //counts _grid voxels with value!=1 and calculates volume. can be used after "fill_unreachable areas" or after "brushfire"
    double workspace_volume_from_samples(double); //gets workspace volume given _container samples(add with "add_point_to_container"). Needs avg manipulability threshold to define singularities

    void generate_forbidden_region(double); //generates forbidden region. Input is width of this region in m.
    void generate_forbidden_region_gradients(int);

    void rebuild_grids(double); //rescale and rebuild grid and container from _points

    //conversions
    Coordinate point_to_coordinate(Point);
    Point coordinate_to_point(Coordinate); //gets point at center of voxel

    void publish_grid(ros::Publisher&,string);

    void publish_sphere_fit(ros::Publisher&);

    ~DiscreteWorkspace();

    vector<vector<vector<Vector>>> _gradients;

    vector<Coordinate> _forbidden_region;

private:
    string _rviz_base_frame="world";

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
    double _maxfire=1.0; //maximum voxel value from brushfire
    double _max_manip=0.0; //maximum manipulability for a voxel
    double _min_manip=1.0; //minimum non-zero manipulability for a voxel

    //data 
    vector<vector<vector<double>>> _grid; //discrete 3D workspace. Singularities = 1, contains brushfire values and is used for visualization
    vector<vector<vector<vector<double>>>> _containergrid; //discrete 3D workspace with list of values at each voxels. Used for averaging.

    vector<Point> _points; //list of added samples, used for rebuild and averaging
    vector<Point> _singularities; //list of singularities, used to merge grids

    Sphere _singularity_fit;

    //methods

    void build_grid(); //build _grid with 0 in each voxel
    void build_container(); //build _containergrid with a 0 in each voxel
    void build_gradients();

    bool in_bounds(int,int,int);
    bool in_bounds(Coordinate);
    bool add_point_to_container_only(Point); //add point to container, used by rebuild (we dont want duplicates in _points)
    //bool add_point_to_grid_only(Point); //add point to grid, used by rebuild (we dont want duplicates in _points)

    void unique_append(vector<Point>&, Point);

    vector<Point> convex_hull_singularities(); //creates convex hull from _singularities. Untested and unused


};


DiscreteWorkspace::DiscreteWorkspace(double x,double y,double z, int res)
{
    _x_size=x;
    _y_size=y;
    _z_size=z;
    _resolution=res;
    build_grid();
    build_container();
    build_gradients();
}

DiscreteWorkspace::DiscreteWorkspace(DiscreteWorkspace& w1,DiscreteWorkspace& w2,int){
    _x_size=w1._x_size;
    _y_size=w1._y_size;
    _z_size=w1._z_size;
    _resolution=w1._resolution;
    build_grid();
    for(int x=0;x<_x_voxels;x++){
        for(int y=0;y<_y_voxels;y++){
            for(int z=0;z<_z_voxels;z++){
                (*this)(x,y,z)=w1(x,y,z) || w2(x,y,z);
            }
        }
    }

    
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

void DiscreteWorkspace::build_gradients(){
    vector<Vector> z_cells(_z_voxels);
    vector<vector<Vector>> y_cells(_y_voxels,z_cells);
    vector<vector<vector<Vector>>> x_cells(_x_voxels,y_cells);
    _gradients=x_cells;
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
    Coordinate c(floor(p.x*_resolution)+round(0.5*_x_voxels),floor(p.y*_resolution)+round(0.5*_y_voxels),floor(p.z*_resolution)+round(0.5*_z_voxels));
    return c;
}

Point DiscreteWorkspace::coordinate_to_point(Coordinate c){
    Point p((c.x-round(0.5*_x_voxels))/_resolution+0.5/_resolution,(c.y-round(0.5*_y_voxels))/_resolution+0.5/_resolution,(c.z-round(0.5*_z_voxels))/_resolution+0.5/_resolution,(*this)(c));
    return p;
}

void DiscreteWorkspace::unique_append(vector<Point>& v, Point p){
    bool exists=0;
    for(int i=0;i<v.size();i++){
        if(v.at(i)==p){
            exists=1;
            break;
        }
    }
    if(!exists){
        v.push_back(p);
    }
}
/*
bool DiscreteWorkspace::add_point_to_grid_only(Point p){
    //assumes points are in [m]
    if(in_bounds(point_to_coordinate(p))){
        (*this)(point_to_coordinate(p))=p.val;   
        return 1;    
    } else {
        return 0;
    }

}
*/
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
/*
bool DiscreteWorkspace::add_point(Point p){
    //assumes points are in [m]
    if(add_point_to_grid_only(p)){
        _points.push_back(p); //add point to _points
        return 1;    
    } else {
        return 0;
    }

}
*/
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
    _max_manip=0; //for visualization 
}

void DiscreteWorkspace::fit_sphere_to_singularities(){

    //parameters
    double sigma=1;

    using namespace util;        
    int N=_singularities.size(); //points
  
    int M=4;


    //generating design matrix A.
    MatDoub A(N,M);
    VecDoub b(N);

    for(int i=0;i<N;i++){
        A[i][0]=2*_singularities.at(i).x;
        A[i][1]=2*_singularities.at(i).y;
        A[i][2]=2*_singularities.at(i).z;
        A[i][3]=1;
        b[i]=(pow(_singularities.at(i).x,2)+pow(_singularities.at(i).y,2)+pow(_singularities.at(i).z,2))/sigma;
    }

    MatDoub AT(M,N);
    MatDoub C(M,M);
    VecDoub c(M);
    VecDoub a(M);

    AT=Transpose(A);  //calculate A transposed using util
    //calculate normal equations C and c
    C=AT*A;
    c=AT*b;
    //solve
    SVD svd(C);
    svd.solve(c,a);
    /*
    cout << "a: " << a[0] << endl;
    cout << "b: " << a[1] << endl;
    cout << "c: " << a[2] << endl;
    cout << "r: " << sqrt(a[3]+pow(a[0],2)+pow(a[1],2)+pow(a[2],2)) << endl;
    */
    _singularity_fit=Sphere(a[0],a[1],a[2],sqrt(a[3]+pow(a[0],2)+pow(a[1],2)+pow(a[2],2)));
}

void DiscreteWorkspace::publish_grid(ros::Publisher &pub, string setting="singularities"){
    

  //MARKER SETUP-------------------------------------------------------------------------------------------------------------------------------------------------------------
    visualization_msgs::Marker marker;
    visualization_msgs::MarkerArray marker_array;

    //delete sphere
    marker.action = visualization_msgs::Marker::ADD;
    marker.header.frame_id = _rviz_base_frame; //SET TO PANDA LINK 0
    marker.header.stamp = ros::Time::now();
    marker.type =  visualization_msgs::Marker::SPHERE;
    marker.ns="singular_sphere_fit";
    marker.id=0;

    marker_array.markers.push_back(marker);

    //delete all cubes before adding new markers
    marker.type =  visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(marker);

    //CREATE OTHER GRID MARKERS------------------------------------------------------------------------------------------------
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = _rviz_base_frame; //SET TO PANDA LINK 0
    marker.header.stamp = ros::Time::now();


    // Set the marker type. 
    marker.type =  visualization_msgs::Marker::CUBE;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
/*
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
*/
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



    //Create marker array with markers based on setting, starts at id 200, so we can use the first 200 for other markers
    int m_id=200;
    marker.ns="voxels";



    marker.action = visualization_msgs::Marker::ADD;

    for(int x=0;x<_x_voxels;x++){
        for(int y=0;y<_y_voxels;y++){
            for(int z=0;z<_z_voxels;z++){
                // Set the namespace and id for this marker.  This serves to create a unique ID
                // Any marker sent with the same namespace and id will overwrite the old one
                marker.id=++m_id;

                //(1.0-(((*this)(x,y,z))/_maxfire));                     
                //ROS_INFO("value: %f maxfire: %f frac: %f",(*this)(x,y,z),_maxfire,marker.color.a);
                if(setting!="gradientarrows"){
                    marker.pose.position.x = coordinate_to_point(Coordinate(x,y,z)).x;
                    marker.pose.position.y = coordinate_to_point(Coordinate(x,y,z)).y;
                    marker.pose.position.z = coordinate_to_point(Coordinate(x,y,z)).z;
                }
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
                    if((*this)(x,y,z)==0){
                        marker.color.b=0;
                        marker.color.g=0;
                        marker.color.r=1;
                    }

                    marker_array.markers.push_back(marker);
                } else if(setting=="singularitygradient"){
                    marker.color.b=min(1*((*this)(x,y,z))/_maxfire,1.0); //max is taken so that gradient works regardless of if brushfire or manipubility grid is being visualized
                    marker.color.g=max(1.0-1*((*this)(x,y,z))/_maxfire,0.0);
 
                    marker_array.markers.push_back(marker);

                } else if(setting=="forbiddenregions"){
                    marker.color.b=min(1*((*this)(x,y,z))/_maxfire,1.0); //max is taken so that gradient works regardless of if brushfire or manipubility grid is being visualized
                    marker.color.g=max(1.0-1*((*this)(x,y,z))/_maxfire,0.0);
                    bool is_forbidden=0;
                    for(int i=0;i<_forbidden_region.size();i++){
                        Coordinate c1=_forbidden_region.at(i);
                        if((c1.x==x)&&(c1.y==y)&&(c1.z==z)){
                            is_forbidden=1;
                            break;
                        }
                    }
                    if(is_forbidden){
                        marker.color.b=0.0f;
                        marker.color.g=0.0f;
                        marker.color.r=1.0f;
                    }
                    marker_array.markers.push_back(marker);

                } else if(setting=="gradientarrows"){
                    marker.type =  visualization_msgs::Marker::ARROW;
                    bool is_forbidden=0;
                    for(int i=0;i<_forbidden_region.size();i++){
                        Coordinate c1=_forbidden_region.at(i);
                        if((c1.x==x)&&(c1.y==y)&&(c1.z==z)){
                            is_forbidden=1;
                            break;
                        }
                    }
                    if(is_forbidden){
                        Vector gradient=_gradients.at(x).at(y).at(z);

                        Point center=coordinate_to_point(Coordinate(x,y,z));
                        //get end and start point for arrow marker
                        Point end=(gradient*0.5)+center;
                        Point start=(gradient*(-0.5))+center;
                        //convert to geometry msgs
                        geometry_msgs::Point p0;
                        p0.x=start.x; p0.y=start.y; p0.z=start.z;
                        geometry_msgs::Point p1;
                        p1.x=end.x; p1.y=end.y; p1.z=end.z;
                        
                        //ROS_INFO("start(%f,%f,%f), end(%f,%f,%f)",p0.x,p0.y,p0.z,p1.x,p1.y,p1.z);
                        ROS_INFO("center(%f,%f,%f)",center.x,center.y,center.z);
                        //add to marker
                        marker.points.clear();
                        marker.points.push_back(p0);
                        marker.points.push_back(p1);

                        marker.scale.x = (1.0/_resolution)*(0.1);
                        marker.scale.y = (1.0/_resolution)*(0.2);
                        marker.scale.z = 0.0;
                        marker.color.b=0.0f;
                        marker.color.g=0.0f;
                        marker.color.r=1.0f;
                        marker_array.markers.push_back(marker);

                    }
                }
            }
        }
    }

                    ROS_INFO("maxfire: %f",_maxfire);

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

void DiscreteWorkspace::publish_sphere_fit(ros::Publisher &pub){
    double radius=_singularity_fit.r;
    double x_center=_singularity_fit.a;
    double y_center=_singularity_fit.b;
    double z_center=_singularity_fit.c;

  //MARKER SETUP-------------------------------------------------------------------------------------------------------------------------------------------------------------
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = _rviz_base_frame; //SET TO PANDA LINK 0
    marker.header.stamp = ros::Time::now();

    // Set the marker type. 
    marker.type =  visualization_msgs::Marker::SPHERE;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.pose.position.x = x_center;
    marker.pose.position.y = y_center;
    marker.pose.position.z = z_center;


    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 2*radius;
    marker.scale.y = 2*radius;
    marker.scale.z = 2*radius; 

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 1.0f;
    marker.color.b = 0.0f;
    marker.color.g = 0.0f;

    marker.color.a = .5;
    marker.lifetime = ros::Duration();

    //Create markyer array with markers based on setting
    int m_id=0;//_x_voxels*_y_voxels*_z_voxels*20;
    marker.ns="singular_sphere_fit";
    //MARKER SETUP-------------------------------------------------------------------------------------------------------------------------------------------------------------

    //spin untill ros is ok, publish array if subscriber is there
    while(true){
        if(ros::ok()){
            if(pub.getNumSubscribers() > 0){
               ROS_INFO("Sphere SUBSCRIBER DETECteD");
                pub.publish(marker);
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

void DiscreteWorkspace::add_singularities_from_fit(){
    Point center(_singularity_fit.a,_singularity_fit.b,_singularity_fit.c,0);
    double r=_singularity_fit.r;
    for(int x=0;x<_x_voxels;x++){
        for(int y=0;y<_y_voxels;y++){
            for(int z=0;z<_z_voxels;z++){ //vector with length r from center pointing towards arbitrary point is closest point on sphere
                Vector v(coordinate_to_point(Coordinate(x,y,z)),center);
                double scalar=r/v.size();
                v.x=scalar*v.x;
                v.y=scalar*v.y;
                v.z=scalar*v.z;
                Point closest_point(center.x+v.x,center.y+v.y,center.z+v.z,1);
                if(in_bounds(point_to_coordinate(closest_point))){
                    (*this)(point_to_coordinate(closest_point))=closest_point.val; 
                }
            }
        }
    }
}

void DiscreteWorkspace::fill_unreachable_areas(){
    vector<Coordinate> newly_updated={Coordinate(0,0,0)};
    int value = 1;
 
    while(newly_updated.size()>0){ //while new vals added
        vector<Coordinate> prev_updated=newly_updated; //set to prev vals
        newly_updated.clear();
        for(int i=0;i<prev_updated.size();i++){ //iterate prev vals
            //check nbs
            for(int x_offset=-1;x_offset<=1;x_offset++){
                for(int y_offset=-1;y_offset<=1;y_offset++){
                    for(int z_offset=-1;z_offset<=1;z_offset++){
                        if((abs(x_offset)+abs(y_offset)+abs(z_offset))==1){  //for 4nbs we only want combinations that offset by 1 along an axis
                            Coordinate nb(prev_updated.at(i).x+x_offset,prev_updated.at(i).y+y_offset,prev_updated.at(i).z+z_offset); 
                            if(in_bounds(nb)){ //check nbs of prev val
                                if(((*this)(nb))==0){ //if in bounds and ==0, apply value 1
                                    (*this)(nb)=value;
                                    newly_updated.push_back(nb);
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

double DiscreteWorkspace::grid_volume(){
    int voxels=0;
    for(int x=0;x<_x_voxels;x++){
        for(int y=0;y<_y_voxels;y++){
            for(int z=0;z<_z_voxels;z++){
                if(((*this)(x,y,z))!=1){
                    voxels++;
                }
            }
        }
    }
    return voxels*pow(1.0/_resolution,3);
}

double DiscreteWorkspace::workspace_volume_from_samples(double threshold){
    generate_average_grid();
    average_to_singular_grid(threshold);
    fit_sphere_to_singularities();
    add_singularities_from_fit();
    fill_unreachable_areas();
    return grid_volume();
}

void DiscreteWorkspace::generate_forbidden_region(double width){
    int threshold=ceil(width/(1./_resolution))+1; //brushfire threshold equivalent to width
    for(int x=0;x<_x_voxels;x++){
        for(int y=0;y<_y_voxels;y++){
            for(int z=0;z<_z_voxels;z++){
                if((((*this)(x,y,z))<=threshold)&&((*this)(x,y,z)>1)){
                   _forbidden_region.push_back(Coordinate(x,y,z));
                }
            }
        }
    }
}

void DiscreteWorkspace::generate_forbidden_region_gradients(int depth){
    for(int i=0;i<_forbidden_region.size();i++){ 

        Vector sum(0,0,0);
        Point base=(coordinate_to_point(_forbidden_region.at(i)));
        base.val=(*this)(_forbidden_region.at(i));

        for(int x_offset=-depth;x_offset<=depth;x_offset++){
            for(int y_offset=-depth;y_offset<=depth;y_offset++){
                for(int z_offset=-depth;z_offset<=depth;z_offset++){
                    Coordinate nb(_forbidden_region.at(i).x+x_offset,_forbidden_region.at(i).y+y_offset,_forbidden_region.at(i).z+z_offset); 
                    if(in_bounds(nb)){         
                        Vector target(coordinate_to_point(nb),base);
                        //ROS_INFO("nb: (%f,%f,%f), base: (%f,%f,%f), Target: (%f,%f,%f)",coordinate_to_point(nb).x,coordinate_to_point(nb).y,coordinate_to_point(nb).z,base.x,base.y,base.z,target.x,target.y,target.z);
                        target=target*((*this)(nb)-base.val);
                        //ROS_INFO("scalar: %f",((*this)(nb)-base.val));
                        //ROS_INFO("Target: (%f,%f,%f)",target.x,target.y,target.z);
                        sum=sum+target;
                        //ROS_INFO("sum: (%f,%f,%f)",target.x,target.y,target.z);
                    }
                }
            }
        }
        sum.normalize();
        sum=sum*(1.0/_resolution); //normalize to voxel size
        _gradients.at(_forbidden_region.at(i).x).at(_forbidden_region.at(i).y).at(_forbidden_region.at(i).z)=sum;
    }
}


vector<Point> DiscreteWorkspace::convex_hull_singularities(){
	using namespace quickhull;
	
    QuickHull<float> qh; // Could be double as well
	std::vector<Vector3<float>> pointCloud;
    
	// Add points to point cloud
    for(int i=0;i<_singularities.size();i++){
        pointCloud.push_back(Vector3<float>(_singularities.at(i).x,_singularities.at(i).y,_singularities.at(i).z));
    }

	auto hull = qh.getConvexHull(pointCloud, true, false);
	const auto& vertexBuffer = hull.getVertexBuffer();

    vector<Point> new_sings;
    for(int i=0;i<vertexBuffer.size();i++){
        new_sings.push_back(Point(vertexBuffer[i].x,vertexBuffer[i].y,vertexBuffer[i].z,1));
    }


    return new_sings;

}



#endif