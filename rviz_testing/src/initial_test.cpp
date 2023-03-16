#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip>

using namespace std;

struct Point{ //point in space
    double x;
    double y;  
    double z;
    double val;
};

struct Coordinate{  //grid index
    int x;
    int y;
    int z;
};


class DiscreteWorkspace{

public:
    DiscreteWorkspace(double,double,double,int); //0 for empty cells

    double & operator()(int,int,int);
    double & operator()(Coordinate);

    bool add_point(Point); //adds point to grid and _points

    void brushfire();
    void cpufire();
    void rebuild_grid(double); //rebuild grid from _points

    Coordinate point_to_coordinate(Point);
    
    ~DiscreteWorkspace();


private:
    double dummy = -500;
    //Physical workspace size
    double _x_size;     //[m]
    double _y_size;
    double _z_size;        

    int _resolution; //[voxels/m]
    //Discretized workspace size
    int _x_voxels;
    int _y_voxels;
    int _z_voxels;
    //data 
    vector<vector<vector<double>>> _grid;
    vector<Point> _points;
    //methods
    void build_grid();
    bool in_bounds(int,int,int);
    bool in_bounds(Coordinate);
    bool add_point_to_grid_only(Point); //add point to grid, used by rebuild (we dont want duplicates in _points)

};

DiscreteWorkspace::DiscreteWorkspace(double x,double y,double z, int res)
{
    _x_size=x;
    _y_size=y;
    _z_size=z;
    _resolution=res;
    build_grid();
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
    Coordinate c;
    c.x = round(p.x*_resolution);
    c.y = round(p.y*_resolution);
    c.z = round(p.z*_resolution);
    return c;
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

bool DiscreteWorkspace::add_point(Point p){
    //assumes points are in [m]
    if(add_point_to_grid_only(p)){
        _points.push_back(p); //add point to _points
        return 1;    
    } else {
        return 0;
    }

}

void DiscreteWorkspace::rebuild_grid(double resolution){
    _resolution=resolution;
    build_grid();
    for(int i=0;i<_points.size();i++){
        add_point_to_grid_only(_points.at(i));
    }
}

void DiscreteWorkspace::cpufire(){

    int new_values_added=1;
    int current_sweep=2;

    while(new_values_added){
        new_values_added=0;
        for(int x=0;x<_x_voxels;x++){
            for(int y=0;y<_y_voxels;y++){
                for(int z=0;z<_z_voxels;z++){
                    if(((*this)(x,y,z))==0){
                        bool has_nb=0;
                        //iterate 8 nbs
                        for(int i=-1;i<=1;i++){
                            for(int j=-1;j<=1;j++){
                                for(int k=-1;k<=1;k++){
                                    if(in_bounds(x+i,y+j,z+k)){
                                        if((((*this)(x+i,y+j,z+k))==(current_sweep-1))){ //if any neighbour is occupied with prev sweep value: add current sweep val to point
                                            has_nb=1;
                                            break;
                                        }
                                    }
                                }
                            }
                        }
                        if(has_nb){
                            (*this)(x,y,z)=current_sweep;
                            new_values_added++;
                        }
                    }
                }
            }
        }
        current_sweep++ ; 
        cout << "sweep: " << current_sweep<< ", values added: " << new_values_added << endl;
    }



}


void DiscreteWorkspace::brushfire(){
    vector<Coordinate> newly_updated;
    //add 1 value voxels to newly_updated
    for(int x=0;x<_x_voxels;x++){
        for(int y=0;y<_y_voxels;y++){
            for(int z=0;z<_z_voxels;z++){
                if(((*this)(x,y,z))==1){
                    Coordinate c;
                    c.x=x;
                    c.y=y;
                    c.z=z;
                    newly_updated.push_back(c);
                }
            }
        }
    }
    int value=2;
    while(newly_updated.size()>0){ //while new vals added
        vector<Coordinate> prev_updated=newly_updated; //set to prev vals
        newly_updated.clear();
        for(int i=0;i<prev_updated.size();i++){ //iterate prev vals
            //check nbs
            for(int x_offset=-1;x_offset<=1;x_offset++){
                for(int y_offset=-1;y_offset<=1;y_offset++){
                    for(int z_offset=-1;z_offset<=1;z_offset++){
                        Coordinate nb; 
                        nb.x=prev_updated.at(i).x+x_offset;
                        nb.y=prev_updated.at(i).y+y_offset;
                        nb.z=prev_updated.at(i).z+z_offset;
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
        cout << "Value: " << value << ", points added: " << newly_updated.size() << endl;
        value++;
    }
}

int main(int argc, char const *argv[])
{   
    //DiscreteWorkspace w1(1,1,1,100);
    
    //resize test
    /*
    Point p1;
    p1.x=0.125;
    p1.y=0.00005;
    p1.z=0.163;
    p1.val=2000;

    cout << "add p1" << endl;
    w1.add_point(p1);

    cout << w1(13,0,16) << endl;

    cout << "rebuild" << endl;
    w1.rebuild_grid(200);

    cout << "print old and new pos" <<endl;
    cout << w1(13,0,16) << endl;
    cout << w1(25,0,33) << endl;
    */

    //Brushfire test
    /*
    DiscreteWorkspace w2(1,1,0.05,20);
    w2(5,5,0)=1;
    w2(14,14,0)=1;
    w2(10,2,0)=1;

    w2.brushfire();

    for(int i=0;i<20;i++){
        for(int j=0;j<20;j++){
            cout << setw(3) << w2(i,j,0);
        }
        cout << endl;
    }
    */

    /*
    DiscreteWorkspace w3(2,2,2,100); //100x100x100=35.6s, 200x200x200: 570.9s
    w3(25,20,10)=1;
    w3(15,95,2)=1;
    w3(60,25,30)=1;
    w3(99,1,0)=1;
    w3(2,65,10)=1;
    w3(55,20,80)=1;
    w3(2,65,100)=1;

    //w3.cpufire();          

    */
    DiscreteWorkspace w4(1,1,1,150); //200x200x200: 22.7s  300x300x300 51.3s 600x600x600: 340s
    w4(25,20,10)=1;
    w4(15,95,2)=1;
    w4(60,25,30)=1;
    w4(99,1,0)=1;
    w4(2,65,10)=1;
    w4(55,20,80)=1;
    w4(2,65,99)=1;
 
    w4.brushfire();  

    return 0;
}
