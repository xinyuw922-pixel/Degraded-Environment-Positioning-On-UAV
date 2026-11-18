// Headless adapter: retain the upstream estimator and bag loader unchanged.
#include <trajlo/core/odometry.h>
#include <trajlo/io/data_loader.h>
#include <trajlo/utils/config.h>

int main(int argc, char** argv) {
  if (argc != 2) return 2;
  traj::TrajConfig config;
  config.load(argv[1]);
  traj::TrajLOdometry odometry(config);
  auto loader = traj::DatasetFactory::GetDatasetIo(config.type);
  loader->laser_queue = &odometry.laser_data_queue;
  odometry.Start();
  loader->start();
  loader->publish(config.dataset_path, config.topic);
  // The upstream destructor joins the estimator after the EOF queue marker.
  return 0;
}
