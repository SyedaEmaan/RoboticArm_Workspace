#include "trajectory_analysis/dynamics.hpp"
#include <pinocchio/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>

static pinocchio::Model model;
static pinocchio::Data data;
static bool loaded = false;

Eigen::VectorXd compute_torques(
    const Eigen::VectorXd& q,
    const Eigen::VectorXd& qd,
    const Eigen::VectorXd& qdd)
{
  if (!loaded)
  {
    pinocchio::urdf::buildModel("../../mycobot_gazebo/urdf/ros2_control/gazebo/mycobot_280.urdf", model);
    data = pinocchio::Data(model);
    loaded = true;
  }

  return pinocchio::rnea(model, data, q, qd, qdd);
}
