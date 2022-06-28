#include <cmath>
#include <thread>
#include <queue>
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
      this->delay_ite = 200;
      this->model = _parent;

      // set initial velocity
      this->tx = 0;
      this->ty = 0;
      this->tz = 0;

      this->dtx = 0;
      this->dty = 0;
      this->dtz = 0;

      this->rv = 0;
      this->drv = 0;

      // init queue
      for (int i=0;i<this->delay_ite;i++) {
        this->qx.push(0);
        this->qy.push(0);
        this->qz.push(0);
        this->qr.push(0);
      }

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
      // update queue (for delay)
      this->qx.push(this->dtx);
      this->qy.push(this->dty);
      this->qz.push(this->dtz);
      this->qr.push(this->drv);
      double dtx = this->qx.front();
      double dty = this->qy.front();
      double dtz = this->qz.front();
      double drv = this->qr.front();
      this->qx.pop();
      this->qy.pop();
      this->qz.pop();
      this->qr.pop();

      // Update hover frame velocity
      this->tx += (dtx-this->tx)/1.3*0.001;
      this->ty += (dty-this->ty)/1.3*0.001;
      this->tz += (dtz-this->tz)/0.3*0.001;
      this->rv += (drv-this->rv)/0.1*0.001;

      // World frame velocity
      ignition::math::Pose3d pose;     
      pose = this->model->WorldPose();
      double yaw = pose.Yaw();

      double tx = this->tx*std::cos(yaw)-this->ty*std::sin(yaw);
      double ty = this->tx*std::sin(yaw)+this->ty*std::cos(yaw);

      // set velocity
      this->model->SetLinearVel(ignition::math::Vector3d(tx, ty, this->tz));
      this->model->SetAngularVel(ignition::math::Vector3d(0, 0, this->rv));
    }

    // Handle an incoming message from ROS
    // \param[in] _msg A double value that is used to set the velocity
    // of the Velodyne.
    public: void OnRosMsg(const geometry_msgs::Twist::ConstPtr &_msg)
    {
      this->dtx = 0.017*_msg->linear.x;
      this->dty = 0.017*_msg->linear.y;
      this->dtz = 0.008*_msg->linear.z;
      this->drv = 0.0143*_msg->angular.z;
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

    private: int delay_ite;

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    // pointer to the current velocity
    private: double tx;
    private: double tz;
    private: double ty;

    // pointer to the desired velocity
    private: double dtx;
    private: double dty;
    private: double dtz;

    // pointer to the current rotational velocity
    private: double rv;

    // pointer to the desired rotational velocity
    private: double drv;

    // queue
    std::queue<double> qx;
    std::queue<double> qy;
    std::queue<double> qz;
    std::queue<double> qr;

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
