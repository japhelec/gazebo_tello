#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo
{
  class ModelPush : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;
      
      this->yaw = 0;
      this->vel = 0;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&ModelPush::OnUpdate, this));
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      // this->yaw += 0.0001;

      // // Apply a small linear velocity to the model.
      // this->vel = (1.7-this->vel)/1.3*0.001 + this->vel;
      // this->model->SetLinearVel(ignition::math::Vector3d(this->vel, 0, 0));

      // ignition::math::Pose3d pose;     
      // pose = this->model->WorldPose();
      // double x = pose.X();
      // double y = pose.Y();
      // double z = pose.Z();
      this->model->SetWorldPose(ignition::math::Pose3d(0,0,0,0.17,0,0));
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    private: float vel;
    private: double yaw;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}
