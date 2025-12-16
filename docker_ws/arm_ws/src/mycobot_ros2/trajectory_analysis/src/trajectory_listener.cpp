#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include "trajectory_analysis/dynamics.hpp"

class TrajectoryListener : public rclcpp::Node
{
public:
  TrajectoryListener() : Node("trajectory_listener")
  {
    sub_ = this->create_subscription<moveit_msgs::msg::DisplayTrajectory>(
      "/display_planned_path",
      10,
      std::bind(&TrajectoryListener::callback, this, std::placeholders::_1)
    );
  }

private:
  void callback(const moveit_msgs::msg::DisplayTrajectory::SharedPtr msg)
  {
    if (msg->trajectory.empty()) return;

    auto traj = msg->trajectory[0].joint_trajectory;

    for (const auto& point : traj.points)
    {
      Eigen::VectorXd q(point.positions.size());
      Eigen::VectorXd qd(point.velocities.size());
      Eigen::VectorXd qdd(point.accelerations.size());

      for (Eigen::Index i = 0; i < q.size(); ++i)
      {
        q[i] = point.positions[i];
        qd[i] = point.velocities[i];
        qdd[i] = point.accelerations[i];
      }

      auto tau = compute_torques(q, qd, qdd);
      Eigen::IOFormat fmt(Eigen::StreamPrecision, Eigen::DontAlignCols);
      std::stringstream ss;
      ss << tau.transpose().format(fmt);
      RCLCPP_INFO(this->get_logger(), "Torques: %s", ss.str().c_str());
    }
  }

  rclcpp::Subscription<moveit_msgs::msg::DisplayTrajectory>::SharedPtr sub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryListener>());
  rclcpp::shutdown();
  return 0;
}

