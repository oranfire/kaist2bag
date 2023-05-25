//
// Created by tao on 7/24/21.
//

#ifndef SRC_STEREO_IMU_CONVERTER_H
#define SRC_STEREO_IMU_CONVERTER_H
#include <string>
#include "converter.h"

namespace kaist2bag {

class StereoImuConverter : public Converter {
public:
    StereoImuConverter(const std::string& dataset_dir, const std::string& save_dir,
                 const std::string& imu_topic, const std::string& left_topic,
                 const std::string& right_topic);
    virtual ~StereoImuConverter() = default;

    int Convert() override;

    std::string default_imu_file = "sensor_data/xsens_imu.csv";
    std::string default_img_time_file = "image/stereo_stamp.csv";
    std::string default_left_dir = "image/stereo_left";
    std::string default_right_dir = "image/stereo_right";

private:
    std::string imu_topic_;
    std::string left_topic_;
    std::string right_topic_;
    std::string bag_name_;
};


} // namespace kaist2bag
#endif //SRC_STEREO_IMU_CONVERTER_H
