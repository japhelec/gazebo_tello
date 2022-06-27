#include <thread>
#include <vector>
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
      this->tv[0] = 0;
      this->tv[1] = 0;
      this->tv[2] = 0;

      this->d_tv[0] = 0;
      this->d_tv[1] = 0;
      this->d_tv[2] = 0;

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
      double* p_d_tv = this->d_tv;
      double* p_tv = this->tv;
      double tmp[3];

      std::transform(p_d_tv,p_d_tv+3,p_tv,tmp,std::minus<double>());
      tmp[0] = tmp[0]/1.3*0.001;
      tmp[1] = tmp[1]/1.3*0.001;
      tmp[2] = tmp[2]/1.3*0.001;
      std::transform(p_tv,p_tv+3,tmp,tmp,std::plus<double>());

      this->tv[0] = tmp[0];
      this->tv[1] = tmp[1];
      this->tv[2] = tmp[2];

      // this->vel = (this->d_vel-this->vel)/1.3*0.001 + this->vel;
      this->model->SetLinearVel(ignition::math::Vector3d(tmp[0], tmp[1], tmp[2]));
    }

    // Handle an incoming message from ROS
    // \param[in] _msg A double value that is used to set the velocity
    // of the Velodyne.
    public: void OnRosMsg(const geometry_msgs::Twist::ConstPtr &_msg)
    {
      // this->d_vel = 0.017*_msg->data;
      this->d_tv[0] = 0.017*_msg->linear.x;
      this->d_tv[1] = 0.017*_msg->linear.y;
      this->d_tv[2] = 0.008*_msg->linear.z;
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
    private: double tv[3];

    // pointer to the desired velocity
    private: double d_tv[3];

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
