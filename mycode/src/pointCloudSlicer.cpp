#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/transform_listener.h"
#include <vector>

using namespace std;

class pointCloudSlicer{
    private:
        ros::Publisher pub;
        ros::Subscriber sub;
        double sliceWidth;
        double slicePos;

        void callback(const sensor_msgs::PointCloud::ConstPtr& msg){
            ROS_INFO("callback");
            vector<geometry_msgs::Point32> points = msg->points;
            sensor_msgs::PointCloud slice;
            sensor_msgs::ChannelFloat32 singular_metric;
            singular_metric.name = "singular metric";

            slice.header.stamp = ros::Time();
            slice.header.frame_id = msg->header.frame_id;
            for (int i = 0; i < points.size(); i++){
                if (abs(points.at(i).x - slicePos) < sliceWidth){
                    slice.points.push_back(points.at(i));
                    singular_metric.values.push_back(msg->channels.at(0).values.at(i));
                }
            }
            slice.channels.push_back(singular_metric);
            pub.publish(slice);
        };
    public:
        pointCloudSlicer(ros::NodeHandle n, string fromTopic, string toTopic, double sliceWidth, double slicePos){
            this->sliceWidth = sliceWidth;
            this->slicePos = slicePos;
            this->pub = n.advertise<sensor_msgs::PointCloud>(toTopic, 1);
            this->sub = n.subscribe<sensor_msgs::PointCloud>(fromTopic, 1, &pointCloudSlicer::callback, this);
        }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pointCloudDuplicator");
    ros::NodeHandle n;
    pointCloudSlicer pctf(n, "robot1/workspacePointCloud", "robot1/workspacePointCloud_slice", 0.1, 0.0);

    ros::spin();

  return 0;
}