#ifndef _SIMPLE_DRIVE_HH
#define _SIMPLE_DRIVE_HH


#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>


#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>

#include <string>
#include <memory>

namespace gazebo
{


  class SimpleDrive : public ModelPlugin
  {

    /// \brief Constructor.
    public: SimpleDrive();

    /// \brief Destructor.
    public: ~SimpleDrive();

    // Documentation Inherited.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Callback for the World Update event.
    public: virtual void OnUpdate();

    public: void CmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);

        /// \brief Pointer to the model containing the plugin.
    protected: physics::ModelPtr drive_rsModel;

    protected: ros::NodeHandle* drive_rosnode_;

    private: ros::Subscriber drive_sub;
    

  };
}
#endif
