#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "geometry_msgs/Twist.h"
#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo
{
  class TelloSim : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

      // set initial velocity
      this->vel = 0;
      this->d_vel = 0;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&TelloSim::OnUpdate, this));

      // Initialize ros, if it has not already bee initialized.
      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_client",
            ros::init_options::NoSigintHandler);
      }

      // Create our ROS node. This acts in a similar manner to
      // the Gazebo node
      this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

      // Create a named topic, and subscribe to it.
      ros::SubscribeOptions so =
        ros::SubscribeOptions::create<geometry_msgs::Twist>(
            "/" + this->model->GetName() + "/vel_cmd",
            1,
            boost::bind(&TelloSim::OnRosMsg, this, _1),
            ros::VoidPtr(), &this->rosQueue);
      this->rosSub = this->rosNode->subscribe(so);

      // Spin up the queue helper thread.
      this->rosQueueThread =
        std::thread(std::bind(&TelloSim::QueueThread, this));
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      // Apply a small linear velocity to the model.
      this->vel = (this->d_vel-this->vel)/1.3*0.001 + this->vel;
      this->model->SetLinearVel(ignition::math::Vector3d(this->vel, 0, 0));
    }

    // Handle an incoming message from ROS
    // \param[in] _msg A float value that is used to set the velocity
    // of the Velodyne.
    public: void OnRosMsg(const geometry_msgs::Twist::ConstPtr &_msg)
    {
      // this->d_vel = 0.017*_msg->data;
      std::cout << _msg->linear.x << std::endl;
    }

    // ROS helper function that processes messages
    private: void QueueThread()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }



    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    // pointer to the current velocity
    private: float vel;

    // pointer to the desired velocity
    private: float d_vel;

    // A node use for ROS transport
    private: std::unique_ptr<ros::NodeHandle> rosNode;

    // A ROS subscriber
    private: ros::Subscriber rosSub;

    // A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue rosQueue;

    // A thread the keeps running the rosQueue
    private: std::thread rosQueueThread;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(TelloSim)
}
