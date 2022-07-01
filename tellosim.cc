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
      this->tello = this->model->GetLink("tello");

      this->dtv = ignition::math::Vector3d(0,0,0);
      this->drv = ignition::math::Vector3d(0,0,0);

      // // init queue
      // for (int i=0;i<this->delay_ite;i++) {
      //   this->qx.push(0);
      //   this->qy.push(0);
      //   this->qz.push(0);
      //   this->qr.push(0);
      // }

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
      // // update queue (for delay)
      // this->qx.push(this->dtv.x);
      // this->qy.push(this->dtv.y);
      // this->qz.push(this->dtv.z);
      // this->qr.push(this->drv.z);

      // ignition::math::Vector3d<double> dtv = ignition::math::Vector3d(
      //   this->qx.front(),
      //   this->qy.front(),
      //   this->qz.front()
      // );

      // ignition::math::Vector3d<double> drv = ignition::math::Vector3d(
      //   0,
      //   0,
      //   this->qr.front()
      // );

      // this->qx.pop();
      // this->qy.pop();
      // this->qz.pop();
      // this->qr.pop();

      // Update hover frame velocity
      // this->tx += (dtx-this->tx)/1.3*0.001;
      // this->ty += (dty-this->ty)/1.3*0.001;
      // this->tz += (dtz-this->tz)/0.3*0.001;
      // this->rv += (drv-this->rv)/0.1*0.001;

      // get current hover velocity
      ignition::math::Pose3d pose;     
      pose = this->model->WorldPose();
      // double yaw = pose.Yaw();

      ignition::math::Vector3d tv = this->model->WorldLinearVel();
      tv = pose.Rot().RotateVectorReverse(tv);
      ignition::math::Vector3d rv = this->model->WorldAngularVel();

      ignition::math::Vector3d tac = (this->dtv - tv)/1.3;
      tac = tac + ignition::math::Vector3d(0,0,9.8);
      tac = pose.Rot().RotateVector(tac);

      ignition::math::Vector3d rac = (this->drv - rv)/0.1;

      // set velocity
      this->tello->AddForce(tac);
      this->tello->AddTorque(rac);
    }

    // Handle an incoming message from ROS
    // \param[in] _msg A double value that is used to set the velocity
    // of the Velodyne.
    public: void OnRosMsg(const geometry_msgs::Twist::ConstPtr &_msg)
    {
      this->dtv = ignition::math::Vector3d(
        0.017*_msg->linear.x,
        0.017*_msg->linear.y,
        0.008*_msg->linear.z
      );
      this->drv = ignition::math::Vector3d(
        0,
        0,
        0.0143*_msg->angular.z
      );
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
    private: physics::LinkPtr tello;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    // pointer to the desired velocity
    private: ignition::math::Vector3d dtv;
    private: ignition::math::Vector3d drv;

    // // queue
    // std::queue<double> qx;
    // std::queue<double> qy;
    // std::queue<double> qz;
    // std::queue<double> qr;

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
