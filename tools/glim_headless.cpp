// Offline CPU odometry adapter using upstream GLIM preprocessing and estimation.
#include <fstream>
#include <iostream>
#include <iomanip>
#include <thread>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <livox_ros_driver2/CustomMsg.h>
#include <glim/util/config.hpp>
#include <glim/util/raw_points.hpp>
#include <glim/preprocess/cloud_preprocessor.hpp>
#include <glim/odometry/async_odometry_estimation.hpp>
#include <glim/odometry/estimation_frame.hpp>

class GlimBagReplay {
 public:
  GlimBagReplay(const std::string& config, const std::string& output)
      : trajectory_(output) {
    glim::GlobalConfig::instance(config);
    auto estimator = glim::OdometryEstimationBase::load_module(
        "libodometry_estimation_cpu.so");
    if (!estimator) throw std::runtime_error("Cannot load CPU odometry module");
    estimator_.reset(new glim::AsyncOdometryEstimation(estimator, true));
    preprocessor_.reset(new glim::CloudPreprocessor);
    trajectory_ << std::setprecision(18);
  }

  void run(const std::string& filename) {
    rosbag::Bag bag(filename, rosbag::bagmode::Read);
    rosbag::View view(bag, rosbag::TopicQuery(std::vector<std::string>{"/livox/lidar", "/livox/imu"}));
    size_t lidar_count = 0, imu_count = 0;
    for (const auto& entry : view) {
      if (entry.getTopic() == "/livox/imu") {
        const auto message = entry.instantiate<sensor_msgs::Imu>();
        const auto& acc = message->linear_acceleration;
        const auto& gyro = message->angular_velocity;
        estimator_->insert_imu(message->header.stamp.toSec(),
            9.80665 * Eigen::Vector3d(acc.x, acc.y, acc.z),
            Eigen::Vector3d(gyro.x, gyro.y, gyro.z));
        ++imu_count;
      } else {
        const auto message = entry.instantiate<livox_ros_driver2::CustomMsg>();
        auto cloud = std::make_shared<glim::RawPoints>();
        cloud->stamp = message->header.stamp.toSec();
        for (const auto& point : message->points) {
          cloud->points.emplace_back(point.x, point.y, point.z, 1.0);
          cloud->times.push_back(point.offset_time * 1e-9);
          cloud->intensities.push_back(point.reflectivity);
        }
        auto preprocessed = preprocessor_->preprocess(cloud);
        if (preprocessed && preprocessed->size() >= 100) {
          while (estimator_->workload() > 10) {
            collect();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
          }
          estimator_->insert_frame(preprocessed);
        }
        if (++lidar_count % 100 == 0)
          std::cout << "Read lidar=" << lidar_count << " imu=" << imu_count << std::endl;
      }
      collect();
    }
    estimator_->join();
    collect();
    std::cout << "Completed lidar=" << lidar_count << " imu=" << imu_count << std::endl;
  }

 private:
  void collect() {
    std::vector<glim::EstimationFrame::ConstPtr> current, marginalized;
    estimator_->get_results(current, marginalized);
    // Save online odometry outputs, without later global graph corrections.
    for (const auto& frame : current) {
      if (!frame) continue;
      const auto& pose = frame->T_world_imu;
      const Eigen::Quaterniond rotation(pose.linear());
      trajectory_ << frame->stamp << ' ' << pose.translation().transpose() << ' '
          << rotation.x() << ' ' << rotation.y() << ' ' << rotation.z() << ' '
          << rotation.w() << '\n';
    }
    trajectory_.flush();
  }

  std::ofstream trajectory_;
  std::unique_ptr<glim::CloudPreprocessor> preprocessor_;
  std::unique_ptr<glim::AsyncOdometryEstimation> estimator_;
};

int main(int argc, char** argv) {
  if (argc != 4) return 2;
  GlimBagReplay(argv[1], argv[3]).run(argv[2]);
  return 0;
}
