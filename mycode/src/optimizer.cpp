#include <ros/ros.h>
#include <cmath>
#include <iostream>
#include <math.h>
#include "Eigen/Dense"
#include <vector>
#include <numeric>
#include "Eigen/src/Core/Matrix.h"
#include "Eigen/src/Core/util/Constants.h"
#include <std_msgs/Int32.h>

using namespace std;
using namespace Eigen;

double Himmelblau(Matrix<double, 2, 1> x){
    // identical local minima:
    // f ( 3.0 , 2.0 ) = 0.0
    // f ( − 2.805118 , 3.131312 ) = 0.0
    // f ( − 3.779310 , − 3.283186 ) = 0.0
    // f ( 3.584428 , − 1.848126 ) = 0.0
    // global maxima:
    // f(−0.270845, −0.923039) = 181.617
    return pow(pow(x(0), 2) + x(1) - 11, 2) + pow(x(0) + pow(x(1), 2) - 7, 2);
}

// T must be able to be called with double operator()(Matrix<double, dim, 1> x)
template <typename T, int dim>
class NelderMead{
    public:
        NelderMead(T f, double alpha = 1.0, double gamma = 2.0, double rho = 0.5, double sigma = 0.5)
        : f(f), ALPHA(alpha), GAMMA(gamma), RHO(rho), SIGMA(sigma) {};
        
        Matrix<double, dim, 1> runFiniteDifferenceInitialization(Matrix<double, dim, 1> x0, double delta = 0.1, double std_dev_crit = 1e-3, int max_iter = 100, double volume_crit = 1e-6){
            Matrix<double, dim, dim+1> s;
            s.col(0) = x0;
            for (int i = 1; i < dim+1; i++){
                Matrix<double, dim, 1> offset;
                offset.setZero();
                offset(i-1) = delta;
                s.col(i) = x0 + offset;
            }
            return run(s, std_dev_crit, max_iter, volume_crit);
        }
        Matrix<double, dim, 1> run(Matrix<double, dim, dim+1> x0, double std_dev_crit = 1e-3, int max_iter = 100, double volume_crit = 1e-6){
            // reset state
            x = x0;
            for (int i = 0; i < dim+1; i++){
                y(i) = f(x.col(i));
            }
            //Run nelder mead
            double std_dev;
            double volume;
            int its = 0;

            while(true){
                next();
                its++;
                // standard deviation
                double mean = y.sum()/(dim+1);
                double sum = 0;
                for (double v : y){
                    sum += pow(v - mean, 2);
                }
                double std_dev = sqrt(sum/(dim));
                
                // simplex volume
                Matrix<double, dim, dim> A;
                for(int i = 0; i < dim; i++){
                    A.col(i) = x.col(i+1) - x.col(0);
                }
                double volume = (1.0/factorial(dim+1))*abs(A.determinant());
                cout << "   volume: " << volume  << "   std_dev: " << std_dev << "  its: " << its << endl;
                
                // termination criteria
                if ((its > max_iter) || (volume < volume_crit) || (std_dev < std_dev_crit)){
                    if (its > max_iter){
                        cout << "reached maximum iterations" << endl;
                    }
                    if (volume < volume_crit){
                        cout << "reached min volume" << endl;
                    }
                    if (std_dev < std_dev_crit){
                        cout << "reached min standard deviation" << endl;
                    }
                    break;
                }
            }
            // Centroid of points
            Matrix<double, dim, 1> centroid = (x.rowwise().sum())/(dim+1);

            return centroid;
        }
        
    private:
        long int factorial (long int n){
            return (n == 1 || n == 0) ? 1 : factorial(n - 1) * n;
        }
        void next(){
            // sorted indices (could be updated)
            std::vector<int> indices(dim+1);
            std::iota(indices.begin(), indices.end(), 0);
            std::sort(indices.begin(), indices.end(),
                [&](int i1, int i2) -> bool {
                        return y(i1) < y(i2);
                });

            // Centroid of best points
            Matrix<double, dim, 1> centroid = (x.rowwise().sum() - x.col(indices.back()))/dim;

            // Reflection
            Matrix<double, dim, 1> reflected = centroid + ALPHA*(centroid - x.col(indices.back()));

            // if better than second worst but not better than best re>place worst point with reflected
            double f_of_reflected = f(reflected);
            if (f_of_reflected < y(indices.end()[-2]) && f_of_reflected > y(indices.at(0))){
                x.col(indices.back()) = reflected;
                y(indices.back()) = f_of_reflected;
                cout << "reflected";
                return;
            }

            // expand
            if (f_of_reflected < y(indices.at(0))){
                Matrix<double, dim, 1> expanded = centroid + GAMMA*(reflected - centroid);
                double f_of_expanded = f(expanded);
                if (f_of_expanded < f_of_reflected){
                    x.col(indices.back()) = expanded;
                    y(indices.back()) = f_of_expanded;
                    cout << "expanded success";
                    return;
                } else {
                    x.col(indices.back()) = reflected;
                    y(indices.back()) = f_of_reflected;
                    cout << "expanded fail: reflected";
                    return;
                }
            }

            // contraction
            if (f_of_reflected < y(indices.back())){
                Matrix<double, dim, 1> contracted = centroid + RHO*(reflected - centroid);
                double f_of_contraction = f(contracted);
                if (f_of_contraction < f_of_reflected){
                    x.col(indices.back()) = contracted;
                    y(indices.back()) = f_of_contraction;
                    cout << "contraction: outside";
                    return;
                } else{
                    //shrink
                    for (int i = 1; i < dim+1; i++){
                        x.col(indices.at(i)) = x.col(indices.at(0)) + SIGMA*(x.col(indices.at(i)) - x.col(indices.at(0)));
                    }
                    cout << "shrink";
                    return;
                }
            }
            if (f_of_reflected >= y(indices.back())){
                Matrix<double, dim, 1> contracted = centroid + RHO*(x.col(indices.back()) - centroid);
                double f_of_contraction = f(contracted);
                if (f_of_contraction < y(indices.back())){
                    x.col(indices.back()) = contracted;
                    y(indices.back()) = f_of_contraction;
                    cout << "contraction: inside";
                    return;
                } else {
                    //shrink
                    for (int i = 1; i < dim+1; i++){
                        x.col(indices.at(i)) = x.col(indices.at(0)) + SIGMA*(x.col(indices.at(i)) - x.col(indices.at(0)));
                    }
                    cout << "shrink";
                    return;
                }
            }
            cout << "ERROR: reached end of next() function" << endl;
        };
        
        Matrix<double, 1, dim+1> y;
        Matrix<double, dim, dim+1> x;
        double ALPHA; // reflection factor
        double GAMMA; // expansion factor
        double RHO; // contraction factor
        double SIGMA; // shrink factor
        T f;
};

class WorkspaceEvaluator{
    private:
        ros::NodeHandle node;
        string rx_topic;
        ros::Publisher pub;
    public:
        WorkspaceEvaluator(ros::NodeHandle node, string rx_topic, string tx_topic) : node(node), rx_topic(rx_topic) {
            //this->pub = node.advertise<some_type>(transmit_topic_name, 1);
        };
        double operator()(Matrix<double, Dynamic, 1> x){
            
            cout << "waiting for value" << endl;
            // recieve value
            boost::shared_ptr<std_msgs::Int32 const> shared_ptr;
            shared_ptr = ros::topic::waitForMessage<std_msgs::Int32>(rx_topic);
            cout << "no longer waiting" << endl;
            if (shared_ptr != NULL){
                cout << "message recived: " << *shared_ptr << endl;
            }

            return (double)(*shared_ptr).data;
        };
};

//void callback(const std_msgs::Int32& msg){

    // send message to Carl with transforms of robots
    // recieve message from Philip
//}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "optimizer");
    ros::NodeHandle n;

    std::cout << "Hello, world!\n";
    //Matrix<double, 2, 1> local_minimum = {3.584428 , -1.848126};
    //cout << Himmelblau(local_minimum) << endl;
//
    //Matrix<double, 2, 3> x0 {{-2.0, -1.9, -2.0}, {-2.0, -2.0, -1.9}};
    //NelderMead<2> nm(Himmelblau);
    //Matrix<double, 2, 1> sol = nm.run(x0);
    //cout << Himmelblau(sol) << endl;
//
    //Matrix<double, 2, 1> x {-2, -2};
    //sol = nm.runFiniteDifferenceInitialization(x);
    //cout <<  sol << "\n" << Himmelblau(sol) << endl;

    //n.subscribe<std_msgs::Int32>("recieve_topic", 1, callback);

    Matrix<double, 2, 1> x {-2, -2};
    WorkspaceEvaluator we(n, "recieve_topic", "transmit_topic");
    NelderMead<WorkspaceEvaluator, 2> nm(we);
    Matrix<double, 2, 1> sol = nm.runFiniteDifferenceInitialization(x);
    cout <<  sol << "\n" << we(sol) << endl;

    ros::spin();

  return 0;
}