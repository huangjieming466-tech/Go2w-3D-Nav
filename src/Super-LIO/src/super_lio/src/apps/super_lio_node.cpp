

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "ros/ROSWrapper.h"
#include "lio/super_lio_reloc.h"


using namespace LI2Sup;

int main(int argc, char** argv){
  rclcpp::init(argc, argv);

  ROSWrapper::Ptr data_wrapper = std::make_shared<ROSWrapper>();
  
  auto lio = std::make_shared<SuperLIO>();
  lio->setROSWrapper(data_wrapper);
  lio->init();

  // A 10 Hz processing timer cannot recover after one slow LiDAR frame: its
  // queue stays behind a 10 Hz MID360 stream indefinitely.  Poll cheaply and
  // consume buffered measurements as soon as a complete LiDAR/IMU pair exists.
  auto timer = data_wrapper->create_wall_timer(
    std::chrono::milliseconds(2),
    [lio]() { lio->process(); }
  );

  rclcpp::spin(data_wrapper);

  lio->saveMap();
  lio->printTimeRecord();

  rclcpp::shutdown();
  return 0;
}
