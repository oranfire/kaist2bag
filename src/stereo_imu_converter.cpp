//
// Created by tao on 7/24/21.
//

#include "stereo_imu_converter.h"

#include <rosbag/bag.h>
#include <sensor_msgs/Imu.h>
#include <boost/filesystem.hpp>
#include <sensor_msgs/Image.h>
#include <boost/filesystem.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

namespace kaist2bag {

StereoImuConverter::StereoImuConverter(const std::string& dataset_dir, const std::string& save_dir,
                            const std::string& imu_topic, const std::string& left_topic,
                            const std::string& right_topic)
        : Converter(dataset_dir, save_dir), imu_topic_(imu_topic), left_topic_(left_topic), right_topic_(right_topic){
    bag_name_ = "stereo_imu.bag";
}

int StereoImuConverter::Convert() {
    CheckAndCreateSaveDir();

    boost::filesystem::path bag_file = boost::filesystem::path(save_dir_) / bag_name_;
    rosbag::Bag bag(bag_file.string(), rosbag::bagmode::Write);
    ROS_INFO("saving %s", bag_file.c_str());

    const std::string imu_data_file = dataset_dir_ + "/" + default_imu_file;
    FILE* fp = fopen(imu_data_file.c_str(), "r");
    sensor_msgs::Imu sensor_msgs_imu;

    int64_t stamp;
    double q_x,q_y,q_z,q_w,x,y,z,g_x,g_y,g_z,a_x,a_y,a_z,m_x,m_y,m_z;
    while (fscanf(fp,"%ld,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",
                  &stamp,&q_x,&q_y,&q_z,&q_w,&x,&y,&z,&g_x,&g_y,&g_z,&a_x,&a_y,&a_z,&m_x,&m_y,&m_z) == 17) {
        sensor_msgs_imu.header.stamp.fromNSec(stamp);
        sensor_msgs_imu.header.frame_id = "imu";
        sensor_msgs_imu.orientation.x = q_x;
        sensor_msgs_imu.orientation.y = q_y;
        sensor_msgs_imu.orientation.z = q_z;
        sensor_msgs_imu.orientation.w = q_w;
        sensor_msgs_imu.angular_velocity.x = g_x;
        sensor_msgs_imu.angular_velocity.y = g_y;
        sensor_msgs_imu.angular_velocity.z = g_z;
        sensor_msgs_imu.linear_acceleration.x = a_x;
        sensor_msgs_imu.linear_acceleration.y = a_y;
        sensor_msgs_imu.linear_acceleration.z = a_z;
        sensor_msgs_imu.orientation_covariance[0] = 3;
        sensor_msgs_imu.orientation_covariance[4] = 3;
        sensor_msgs_imu.orientation_covariance[8] = 3;
        sensor_msgs_imu.angular_velocity_covariance[0] = 3;
        sensor_msgs_imu.angular_velocity_covariance[4] = 3;
        sensor_msgs_imu.angular_velocity_covariance[8] = 3;
        sensor_msgs_imu.linear_acceleration_covariance[0] = 3;
        sensor_msgs_imu.linear_acceleration_covariance[4] = 3;
        sensor_msgs_imu.linear_acceleration_covariance[8] = 3;

        bag.write(imu_topic_, sensor_msgs_imu.header.stamp, sensor_msgs_imu);
    }
    fclose(fp);

    const std::string img_time_file = dataset_dir_ + "/" + default_img_time_file;
    const std::string left_dir = dataset_dir_ + "/" + default_left_dir;
    const std::string right_dir = dataset_dir_ + "/" + default_right_dir;
    fp = fopen(img_time_file.c_str(), "r");
    std::vector<int64_t> all_stamps;
    while (fscanf(fp, "%ld\n", &stamp) == 1) {
        all_stamps.push_back(stamp);
    }
    fclose(fp);

    size_t total = all_stamps.size();
    for (size_t i = 0; i < all_stamps.size(); ++i) {
        std::string st = std::to_string(all_stamps[i]);
        
        std::string frame_file = left_dir + "/" + st + ".png";
        // ROS_INFO("converting %s\n", frame_file.c_str());
        if (!boost::filesystem::exists(frame_file)) {
            ROS_WARN("%s not exist\n", frame_file.c_str());
            continue;
        }
        cv::Mat img = cv::imread(frame_file, CV_LOAD_IMAGE_ANYDEPTH);
        cv_bridge::CvImage img_msg;
        img_msg.header.stamp.fromNSec(all_stamps[i]);
        img_msg.header.frame_id = "/"+default_left_dir;
        img_msg.encoding = sensor_msgs::image_encodings::BAYER_BGGR8;
        img_msg.image = img;
        sensor_msgs::Image msg;
        img_msg.toImageMsg(msg);
        bag.write(left_topic_, msg.header.stamp, msg);

        frame_file = right_dir + "/" + st + ".png";
        // ROS_INFO("converting %s\n", frame_file.c_str());
        if (!boost::filesystem::exists(frame_file)) {
            ROS_WARN("%s not exist\n", frame_file.c_str());
            continue;
        }
        img = cv::imread(frame_file, CV_LOAD_IMAGE_ANYDEPTH);
        img_msg.header.stamp.fromNSec(all_stamps[i]);
        img_msg.header.frame_id = "/"+default_right_dir;
        img_msg.encoding = sensor_msgs::image_encodings::BAYER_BGGR8;
        img_msg.image = img;
        img_msg.toImageMsg(msg);
        bag.write(right_topic_, msg.header.stamp, msg);

        // ROS_INFO("total %lu, already convert %lu, remain %lu\n", total, i + 1, total - i - 1);
    }
    
    bag.close();
    ROS_INFO("done saving %s", bag_file.c_str());

    return 0;
}

} // namespace kaist2bag