#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/transform_listener.h"
#include <vector>

using namespace std;

class pointCloudDuplicator{
    private:
        tf::TransformListener listener;
        ros::Publisher pub;
        ros::Subscriber sub;
        string target_frame;

        // move recieved point cloud to target_frame and then transform it to get coordinates in terms of the original frame
        void callback(const sensor_msgs::PointCloud::ConstPtr& msg){
            ROS_INFO("callback");
            sensor_msgs::PointCloud mut_msg(*msg); // Make message mutable, so header frame_id can be changed
            string original_frame = mut_msg.header.frame_id; // save the old frame to use as a transform target
            mut_msg.header.frame_id = target_frame; // give the recieved point cloud a new frame i.e. move it to another frame

            sensor_msgs::PointCloud new_pc(mut_msg); // make another message which will be transformed
            
            if (listener.canTransform(original_frame, target_frame, ros::Time(0))){
                listener.transformPointCloud(original_frame, mut_msg, new_pc);
                pub.publish(new_pc);
            } else {
                ROS_ERROR("listener does not have the propper transforms");
            }
        };
    public:
        pointCloudDuplicator(ros::NodeHandle n, string target_frame, string fromTopic, string toTopic){
            this->target_frame = target_frame;
            this->pub = n.advertise<sensor_msgs::PointCloud>(toTopic, 1);
            this->sub = n.subscribe<sensor_msgs::PointCloud>(fromTopic, 1, &pointCloudDuplicator::callback, this);
        }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "point cloud duplicator");
    ros::NodeHandle n;
    string target_frame, fromTopic, toTopic;
    if (!n.getParam("target_frame", target_frame)){
        ROS_ERROR("parameter target_frame does not exist");
    }
    if (!n.getParam("sub_topic", fromTopic)){
        ROS_ERROR("parameter sub_topic does not exist");
    }
        if (!n.getParam("pub_topic", toTopic)){
        ROS_ERROR("parameter pub_topic does not exist");
    }
    pointCloudDuplicator pctf(n, target_frame, fromTopic, toTopic);

    ros::spin();

  return 0;
}
