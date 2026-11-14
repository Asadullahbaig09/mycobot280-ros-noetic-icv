// Gazebo world plugin: welds a cube to the gripper with a fixed joint.
//
// The myCobot gripper's finger collisions are tiny cylinders, so a pure
// friction grasp in ODE slips. Like most Gazebo pick-and-place demos we
// close the fingers and then attach the object with a fixed joint.
//
//   rostopic pub /vision_pick/attach std_msgs/String "data: 'cube_red'"   # attach
//   rostopic pub /vision_pick/attach std_msgs/String "data: ''"           # detach
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <memory>
#include <mutex>

namespace gazebo
{
class GripperAttachPlugin : public WorldPlugin
{
public:
  void Load(physics::WorldPtr world, sdf::ElementPtr sdf) override
  {
    world_ = world;
    robot_ = sdf->HasElement("robot") ? sdf->Get<std::string>("robot") : "robot";
    // gripper_base is merged into joint6_flange when URDF fixed joints are lumped
    link_ = sdf->HasElement("link") ? sdf->Get<std::string>("link") : "joint6_flange";

    if (!ros::isInitialized())
    {
      gzerr << "GripperAttachPlugin: ROS not initialised, load gazebo via gazebo_ros\n";
      return;
    }
    nh_.reset(new ros::NodeHandle("vision_pick"));
    sub_ = nh_->subscribe("attach", 10, &GripperAttachPlugin::OnMsg, this);
    update_ = event::Events::ConnectWorldUpdateBegin(std::bind(&GripperAttachPlugin::OnUpdate, this));
  }

private:
  void OnMsg(const std_msgs::String::ConstPtr& msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_ = msg->data;
    has_pending_ = true;
  }

  // Joints are created/removed inside the physics loop, never from the ROS thread.
  void OnUpdate()
  {
    std::string target;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!has_pending_)
        return;
      target = pending_;
      has_pending_ = false;
    }

    if (joint_)
    {
      joint_->Detach();
      joint_.reset();
      ROS_INFO("GripperAttachPlugin: detached");
    }
    if (target.empty())
      return;

    physics::ModelPtr robot = world_->ModelByName(robot_);
    physics::ModelPtr object = world_->ModelByName(target);
    physics::LinkPtr parent = robot ? robot->GetLink(link_) : nullptr;
    if (!parent || !object || object->GetLinks().empty())
    {
      ROS_ERROR("GripperAttachPlugin: cannot attach '%s' to %s::%s", target.c_str(), robot_.c_str(), link_.c_str());
      return;
    }
    physics::LinkPtr child = object->GetLinks().front();

    // A fixed joint keeps the relative pose the two links have right now.
    joint_ = world_->Physics()->CreateJoint("fixed", robot);
    joint_->Load(parent, child, ignition::math::Pose3d());
    joint_->Attach(parent, child);
    joint_->Init();
    ROS_INFO("GripperAttachPlugin: attached %s", target.c_str());
  }

  physics::WorldPtr world_;
  physics::JointPtr joint_;
  std::string robot_, link_, pending_;
  bool has_pending_ = false;
  std::mutex mutex_;
  std::unique_ptr<ros::NodeHandle> nh_;
  ros::Subscriber sub_;
  event::ConnectionPtr update_;
};
GZ_REGISTER_WORLD_PLUGIN(GripperAttachPlugin)
}  // namespace gazebo
