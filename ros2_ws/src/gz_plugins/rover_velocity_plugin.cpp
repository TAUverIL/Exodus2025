#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <thread>

namespace gazebo
{
  class RoverVelocityPlugin : public ModelPlugin
  {
  public:
    RoverVelocityPlugin() : ModelPlugin() {}

    void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
    {
      this->model = model;

      // Initialize ROS node
      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = nullptr;
        ros::init(argc, argv, "rover_velocity_plugin",
                  ros::init_options::NoSigintHandler);
      }

      this->rosNode.reset(new ros::NodeHandle("~"));

      // Subscribe to cmd_vel topic
      this->rosSub = this->rosNode->subscribe<geometry_msgs::Twist>(
          "/rover/cmd_vel", 1, &RoverVelocityPlugin::OnVelMsg, this);

      // Retrieve joint names from SDF
      if (sdf->HasElement("left_wheel_joint") && sdf->HasElement("right_wheel_joint"))
      {
        this->leftWheelJointName = sdf->Get<std::string>("left_wheel_joint");
        this->rightWheelJointName = sdf->Get<std::string>("right_wheel_joint");
      }
      else
      {
        gzerr << "[RoverVelocityPlugin] Missing <left_wheel_joint> or <right_wheel_joint> in SDF.\n";
        return;
      }

      this->leftWheelJoint = this->model->GetJoint(this->leftWheelJointName);
      this->rightWheelJoint = this->model->GetJoint(this->rightWheelJointName);

      if (!this->leftWheelJoint || !this->rightWheelJoint)
      {
        gzerr << "[RoverVelocityPlugin] Failed to find joints.\n";
        return;
      }

      gzdbg << "[RoverVelocityPlugin] Plugin loaded successfully.\n";

      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&RoverVelocityPlugin::OnUpdate, this));
    }

    void OnVelMsg(const geometry_msgs::Twist::ConstPtr &msg)
    {
      // Set desired wheel velocities based on cmd_vel message
      this->leftWheelVelocity = msg->linear.x - msg->angular.z;
      this->rightWheelVelocity = msg->linear.x + msg->angular.z;
    }

    void OnUpdate()
    {
      // Apply velocities to joints
      if (this->leftWheelJoint && this->rightWheelJoint)
      {
        this->leftWheelJoint->SetVelocity(0, this->leftWheelVelocity);
        this->rightWheelJoint->SetVelocity(0, this->rightWheelVelocity);
      }
    }

  private:
    physics::ModelPtr model;
    physics::JointPtr leftWheelJoint;
    physics::JointPtr rightWheelJoint;
    std::unique_ptr<ros::NodeHandle> rosNode;
    ros::Subscriber rosSub;
    event::ConnectionPtr updateConnection;

    std::string leftWheelJointName;
    std::string rightWheelJointName;
    double leftWheelVelocity = 0.0;
    double rightWheelVelocity = 0.0;
  };

  GZ_REGISTER_MODEL_PLUGIN(RoverVelocityPlugin)
}
