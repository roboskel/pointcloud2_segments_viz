#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/colors.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl/impl/point_types.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pointcloud_msgs/PointCloud2_Segments.h>

ros::Publisher pub;

std::vector<uint> red = {  0,   0,   255, 255, 255, 102, 102, 204, 0,   255, 52,  152, 152, 0,   0,   204, 0, 255};
std::vector<uint> green = {0,   255, 0,   255, 255, 102, 102, 0,   255, 152, 152, 52,  255, 204, 152, 152, 0, 0};
std::vector<uint> blue = { 255, 0,   0,   0,   255, 152, 52,  152, 255, 52,  255, 255, 52,  152, 204, 0,   0, 255};

std::map<int, uint> id_colour;


void pc2s_callback (const pointcloud_msgs::PointCloud2_Segments& msg){

    sensor_msgs::PointCloud2 accumulator;

    sensor_msgs::PointCloud2 cluster_msgs;

    std::vector<int> to_erase;
    for (auto it = id_colour.begin(); it != id_colour.end(); ++it){
        bool found = false;
        for (size_t i=0; i < msg.cluster_id.size(); i++){
            if (it->first == msg.cluster_id[i]){
                found = true;
                break;
            }
        }
        if (!found){
            to_erase.push_back(it->first);
        }
    }
    for (auto i : to_erase) {
        id_colour.erase(i);
    }

    for (size_t i=0; i < msg.clusters.size(); i++){

        pcl::PCLPointCloud2 cloud2;
        pcl_conversions::toPCL( msg.clusters[i] , cloud2);

        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        pcl::fromPCLPointCloud2(cloud2, cloud);

        if (msg.cluster_id.size() > 0){
            uint c = 0;
            if (id_colour.count(msg.cluster_id[i]) > 0) {
                c = id_colour[msg.cluster_id[i]];
            }
            else{ // Find next unused colour
                for (uint colour=0; colour < red.size(); colour++ ){
                    bool found = false;
                    for (auto it = id_colour.begin(); it != id_colour.end(); ++it){
                        if (it->second == colour) {
                            found = true;
                            break;
                        }
                    }
                    if (!found){
                        c = colour;
                        id_colour[msg.cluster_id[i]] = c;
                        break;
                    }
                }
            }

            for(size_t j=0; j < cloud.points.size(); j++){
                cloud.points[j].r = red[c];
                cloud.points[j].g = green[c];
                cloud.points[j].b = blue[c];
            }
        }
        else { // Randomly draw if there are no ids
            for(size_t j=0; j < cloud.points.size(); j++){
                uint mod = i % red.size();
                cloud.points[j].r = red[mod];
                cloud.points[j].g = green[mod];
                cloud.points[j].b = blue[mod];
            }
        }

        pcl::PCLPointCloud2 clouds;
        pcl::toPCLPointCloud2(cloud, clouds);
        pcl_conversions::fromPCL(clouds, cluster_msgs);
        cluster_msgs.header.stamp = ros::Time::now();
        cluster_msgs.header.frame_id = msg.header.frame_id;

        sensor_msgs::PointCloud2 tmp = sensor_msgs::PointCloud2(accumulator);

        pcl::concatenatePointCloud( cluster_msgs, tmp, accumulator);
    }

    pub.publish(accumulator);
}

int main (int argc, char** argv){
    ros::init (argc, argv, "pointcloud2_segments_viz");
    ros::NodeHandle n_;
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    std::string input_topic;
    std::string out_topic;
    n_.param("pointcloud2_segments_viz/input_topic",input_topic, std::string("/new_pcl"));
    n_.param("pointcloud2_segments_viz/out_topic", out_topic, std::string("pointcloud2_segments_viz/pointcloud2"));

    ros::Subscriber sub = n_.subscribe (input_topic, 1, pc2s_callback);

    pub = n_.advertise<sensor_msgs::PointCloud2> (out_topic, 1);

    ros::spin ();
}
