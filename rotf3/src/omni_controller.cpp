#include <algorithm>
#include <assert.h>

#include "omni_controller.h"

#ifdef ENABLE_PROFILER
#include <ignition/common/Profiler.hh>
#endif

#include <ignition/math/Angle.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>
#include <sdf/sdf.hh>

#include <ros/ros.h>

namespace gazebo
{

enum {
    JOINT1,
    JOINT2,
    JOINT3,
};

GazeboRosOmniDrive::GazeboRosOmniDrive() {}

// Destructor
GazeboRosOmniDrive::~GazeboRosOmniDrive()
{
    FiniChild();
}

// Load the controller
void GazeboRosOmniDrive::Load ( physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{

    this->parent = _parent;
    gazebo_ros_ = GazeboRosPtr ( new GazeboRos ( _parent, _sdf, "OmniDrive" ) );
    // Make sure the ROS node for Gazebo has already been initialized
    gazebo_ros_->isInitialized();

    gazebo_ros_->getParameter<std::string> ( command_topic_, "commandTopic", "cmd_vel" );
    gazebo_ros_->getParameter<std::string> ( odometry_topic_, "odometryTopic", "odom" );
    gazebo_ros_->getParameter<std::string> ( odometry_frame_, "odometryFrame", "odom" );
    gazebo_ros_->getParameter<std::string> ( robot_base_frame_, "robotBaseFrame", "base_footprint" );
    gazebo_ros_->getParameterBoolean ( publishWheelTF_, "publishWheelTF", false );
    gazebo_ros_->getParameterBoolean ( publishOdomTF_, "publishOdomTF", true);
    gazebo_ros_->getParameterBoolean ( publishWheelJointState_, "publishWheelJointState", false );
    // gazebo_ros_->getParameter<double> ( omni_radii_, "omni_radii", 0.04 );
    gazebo_ros_->getParameter<double> ( omni_radii_, "omni_radii", 0.208);
    // gazebo_ros_->getParameter<double> ( wheel_diameter_, "wheelDiameter", 0.026408824 );
    gazebo_ros_->getParameter<double> ( wheel_diameter_, "wheelDiameter", 0.073118483 );
    gazebo_ros_->getParameter<double> ( wheel_accel, "wheelAcceleration", 0.0 );
    gazebo_ros_->getParameter<double> ( wheel_torque, "wheelTorque", 5.0 );
    gazebo_ros_->getParameter<double> ( update_rate_, "updateRate", 100.0 );
    std::map<std::string, OdomSource> odomOptions;
    odomOptions["encoder"] = ENCODER;
    odomOptions["world"] = WORLD;
    gazebo_ros_->getParameter<OdomSource> ( odom_source_, "odometrySource", odomOptions, WORLD );


    joints_.resize ( 3 );
    joints_[JOINT1] = gazebo_ros_->getJoint ( parent, "joint1", "rim_one_joint" );
    joints_[JOINT2] = gazebo_ros_->getJoint ( parent, "joint2", "rim_two_joint" );
    joints_[JOINT3] = gazebo_ros_->getJoint ( parent, "joint3", "rim_three_joint" );
    joints_[JOINT1]->SetParam ( "fmax", 0, wheel_torque );
    joints_[JOINT2]->SetParam ( "fmax", 0, wheel_torque );
    joints_[JOINT3]->SetParam ( "fmax", 0, wheel_torque );



    this->publish_tf_ = true;
    if (!_sdf->HasElement("publishTf")) {
      ROS_WARN_NAMED("omni_drive", "GazeboRosOmniDrive Plugin (ns = %s) missing <publishTf>, defaults to %d",
          this->robot_namespace_.c_str(), this->publish_tf_);
    } else {
      this->publish_tf_ = _sdf->GetElement("publishTf")->Get<bool>();
    }

    // Initialize update rate stuff
    if ( this->update_rate_ > 0.0 ) this->update_period_ = 1.0 / this->update_rate_;
    else this->update_period_ = 0.0;
#if GAZEBO_MAJOR_VERSION >= 8
    last_update_time_ = parent->GetWorld()->SimTime();
#else
    last_update_time_ = parent->GetWorld()->GetSimTime();
#endif

    // Initialize velocity stuff
    wheel_speed_[JOINT1] = 0;
    wheel_speed_[JOINT2] = 0;
    wheel_speed_[JOINT3] = 0;

    // Initialize velocity support stuff
    wheel_speed_instr_[JOINT1] = 0;
    wheel_speed_instr_[JOINT2] = 0;
    wheel_speed_instr_[JOINT3] = 0;

    x_ = 0;
    y_ = 0;
    rot_ = 0;
    alive_ = true;


    if (this->publishWheelJointState_)
    {
        joint_state_publisher_ = gazebo_ros_->node()->advertise<sensor_msgs::JointState>("joint_states", 1000);
        ROS_INFO_NAMED("omni_drive", "%s: Advertise joint_states", gazebo_ros_->info());
    }

    transform_broadcaster_ = boost::shared_ptr<tf::TransformBroadcaster>(new tf::TransformBroadcaster());

    // ROS: Subscribe to the velocity command topic (usually "cmd_vel")
    ROS_INFO_NAMED("omni_drive", "%s: Try to subscribe to %s", gazebo_ros_->info(), command_topic_.c_str());

    ros::SubscribeOptions so =
        ros::SubscribeOptions::create<geometry_msgs::Twist>(command_topic_, 1,
                boost::bind(&GazeboRosOmniDrive::cmdVelCallback, this, _1),
                ros::VoidPtr(), &queue_);

    cmd_vel_subscriber_ = gazebo_ros_->node()->subscribe(so);
    ROS_INFO_NAMED("omni_drive", "%s: Subscribe to %s", gazebo_ros_->info(), command_topic_.c_str());

    if (this->publish_tf_)
    {
      odometry_publisher_ = gazebo_ros_->node()->advertise<nav_msgs::Odometry>(odometry_topic_, 1);
      ROS_INFO_NAMED("omni_drive", "%s: Advertise odom on %s ", gazebo_ros_->info(), odometry_topic_.c_str());
    }

    // start custom queue for omni drive
    this->callback_queue_thread_ =
        boost::thread ( boost::bind ( &GazeboRosOmniDrive::QueueThread, this ) );

    // listen to the update event (broadcast every simulation iteration)
    this->update_connection_ =
        event::Events::ConnectWorldUpdateBegin ( boost::bind ( &GazeboRosOmniDrive::UpdateChild, this ) );

}

void GazeboRosOmniDrive::Reset()
{
#if GAZEBO_MAJOR_VERSION >= 8
  last_update_time_ = parent->GetWorld()->SimTime();
#else
  last_update_time_ = parent->GetWorld()->GetSimTime();
#endif
  pose_encoder_.x = 0;
  pose_encoder_.y = 0;
  pose_encoder_.theta = 0;
  x_ = 0;
  y_ = 0;
  rot_ = 0;
  joints_[JOINT1]->SetParam ( "fmax", 0, wheel_torque );
  joints_[JOINT2]->SetParam ( "fmax", 0, wheel_torque );
  joints_[JOINT3]->SetParam ( "fmax", 0, wheel_torque );
}

void GazeboRosOmniDrive::publishWheelJointState()
{
    ros::Time current_time = ros::Time::now();

    joint_state_.header.stamp = current_time;
    joint_state_.name.resize ( joints_.size() );
    joint_state_.position.resize ( joints_.size() );

    for ( int i = 0; i < 3; i++ ) {
        physics::JointPtr joint = joints_[i];
#if GAZEBO_MAJOR_VERSION >= 8
        double position = joint->Position ( 0 );
#else
        double position = joint->GetAngle ( 0 ).Radian();
#endif
        joint_state_.name[i] = joint->GetName();
        joint_state_.position[i] = position;
    }
    joint_state_publisher_.publish ( joint_state_ );
}
 
void GazeboRosOmniDrive::publishWheelTF()
{
    ros::Time current_time = ros::Time::now();
    for ( int i = 0; i < 3; i++ ) {

        std::string wheel_frame = gazebo_ros_->resolveTF(joints_[i]->GetChild()->GetName ());
        std::string wheel_parent_frame = gazebo_ros_->resolveTF(joints_[i]->GetParent()->GetName ());

#if GAZEBO_MAJOR_VERSION >= 8
        ignition::math::Pose3d poseWheel = joints_[i]->GetChild()->RelativePose();
#else
        ignition::math::Pose3d poseWheel = joints_[i]->GetChild()->GetRelativePose().Ign();
#endif

        tf::Quaternion qt ( poseWheel.Rot().X(), poseWheel.Rot().Y(), poseWheel.Rot().Z(), poseWheel.Rot().W() );
        tf::Vector3 vt ( poseWheel.Pos().X(), poseWheel.Pos().Y(), poseWheel.Pos().Z() );

        tf::Transform tfWheel ( qt, vt );
        transform_broadcaster_->sendTransform (
            tf::StampedTransform ( tfWheel, current_time, wheel_parent_frame, wheel_frame ) );
    }
}

// Update the controller
void GazeboRosOmniDrive::UpdateChild()
{
#ifdef ENABLE_PROFILER
  IGN_PROFILE("GazeboRosOmniDrive::UpdateChild");
  IGN_PROFILE_BEGIN("update");
#endif
    for ( int i = 0; i < 3; i++ ) {
      if ( fabs(wheel_torque -joints_[i]->GetParam ( "fmax", 0 )) > 1e-6 ) {
        joints_[i]->SetParam ( "fmax", 0, wheel_torque );
      }
    }


    if ( odom_source_ == ENCODER ) UpdateOdometryEncoder();
#if GAZEBO_MAJOR_VERSION >= 8
    common::Time current_time = parent->GetWorld()->SimTime();
#else
    common::Time current_time = parent->GetWorld()->GetSimTime();
#endif
    double seconds_since_last_update = ( current_time - last_update_time_ ).Double();

    if ( seconds_since_last_update > update_period_ ) {
        if (this->publish_tf_) publishOdometry ( seconds_since_last_update );
        if ( publishWheelTF_ ) publishWheelTF();
        if ( publishWheelJointState_ ) publishWheelJointState();

        // Update robot in case new velocities have been requested
        getWheelVelocities();

        double current_speed[3];

        current_speed[JOINT1] = joints_[JOINT1]->GetVelocity ( 0 ) * ( wheel_diameter_ / 2.0 );
        current_speed[JOINT2] = joints_[JOINT2]->GetVelocity ( 0 ) * ( wheel_diameter_ / 2.0 );
        current_speed[JOINT3] = joints_[JOINT3]->GetVelocity ( 0 ) * ( wheel_diameter_ / 2.0 );

        if ( wheel_accel == 0 ||
                ( fabs ( wheel_speed_[JOINT1] - current_speed[JOINT1] ) < 0.01 ) ||
                ( fabs ( wheel_speed_[JOINT2] - current_speed[JOINT2] ) < 0.01 ) ||
                ( fabs ( wheel_speed_[JOINT3] - current_speed[JOINT3] ) < 0.01 ) ) {
            //if max_accel == 0, or target speed is reached
            joints_[JOINT1]->SetParam ( "vel", 0, wheel_speed_[JOINT1]/ ( wheel_diameter_ / 2.0 ) );
            joints_[JOINT2]->SetParam ( "vel", 0, wheel_speed_[JOINT2]/ ( wheel_diameter_ / 2.0 ) );
            joints_[JOINT3]->SetParam ( "vel", 0, wheel_speed_[JOINT3]/ ( wheel_diameter_ / 2.0 ) );
        } else {
            if ( wheel_speed_[JOINT1]>=current_speed[JOINT1] )
                wheel_speed_instr_[JOINT1]+=fmin ( wheel_speed_[JOINT1]-current_speed[JOINT1],  wheel_accel * seconds_since_last_update );
            else
                wheel_speed_instr_[JOINT1]+=fmax ( wheel_speed_[JOINT1]-current_speed[JOINT1], -wheel_accel * seconds_since_last_update );

            if ( wheel_speed_[JOINT2]>current_speed[JOINT2] )
                wheel_speed_instr_[JOINT2]+=fmin ( wheel_speed_[JOINT2]-current_speed[JOINT2], wheel_accel * seconds_since_last_update );
            else
                wheel_speed_instr_[JOINT2]+=fmax ( wheel_speed_[JOINT2]-current_speed[JOINT2], -wheel_accel * seconds_since_last_update );

            if ( wheel_speed_[JOINT3]>current_speed[JOINT3] )
                wheel_speed_instr_[JOINT3]+=fmin ( wheel_speed_[JOINT3]-current_speed[JOINT3], wheel_accel * seconds_since_last_update );
            else
                wheel_speed_instr_[JOINT3]+=fmax ( wheel_speed_[JOINT3]-current_speed[JOINT3], -wheel_accel * seconds_since_last_update );
            // ROS_INFO_NAMED("omni_drive", "actual wheel speed = %lf, issued wheel speed= %lf", current_speed[LEFT], wheel_speed_[LEFT]);
            // ROS_INFO_NAMED("omni_drive", "actual wheel speed = %lf, issued wheel speed= %lf", current_speed[RIGHT],wheel_speed_[RIGHT]);

            joints_[JOINT1]->SetParam ( "vel", 0, wheel_speed_instr_[JOINT1] / ( wheel_diameter_ / 2.0 ) );
            joints_[JOINT2]->SetParam ( "vel", 0, wheel_speed_instr_[JOINT2] / ( wheel_diameter_ / 2.0 ) );
            joints_[JOINT3]->SetParam ( "vel", 0, wheel_speed_instr_[JOINT3] / ( wheel_diameter_ / 2.0 ) );
        }
        last_update_time_+= common::Time ( update_period_ );
    }
#ifdef ENABLE_PROFILER
    IGN_PROFILE_END();
#endif
}

// Finalize the controller
void GazeboRosOmniDrive::FiniChild()
{
    alive_ = false;
    queue_.clear();
    queue_.disable();
    gazebo_ros_->node()->shutdown();
    callback_queue_thread_.join();
}

void GazeboRosOmniDrive::getWheelVelocities()
{
    boost::mutex::scoped_lock scoped_lock ( lock );

    double vx = x_;
    double vy = y_;
    double va = rot_;

    // wheel_speed_[LEFT] = vr - va * wheel_separation_ / 2.0;
    // wheel_speed_[RIGHT] = vr + va * wheel_separation_ / 2.0;
    wheel_speed_[JOINT1] = -rot_*omni_radii_ - vy;
    wheel_speed_[JOINT2] = -rot_*omni_radii_ + vy/2 + vx * sqrt(3)/2;
    wheel_speed_[JOINT3] = -rot_*omni_radii_ + vy/2 - vx * sqrt(3)/2;
    // wheel_speed_[JOINT1] = rot_*omni_radii_ + vy;
    // wheel_speed_[JOINT2] = rot_*omni_radii_ - vy/2 - vx * sqrt(3)/2;
    // wheel_speed_[JOINT3] = rot_*omni_radii_ - vy/2 + vx * sqrt(3)/2;
}

void GazeboRosOmniDrive::cmdVelCallback ( const geometry_msgs::Twist::ConstPtr& cmd_msg )
{
    boost::mutex::scoped_lock scoped_lock ( lock );
    x_ = cmd_msg->linear.x;
    y_ = cmd_msg->linear.y;
    rot_ = cmd_msg->angular.z;
}

void GazeboRosOmniDrive::QueueThread()
{
    static const double timeout = 0.01;

    while ( alive_ && gazebo_ros_->node()->ok() ) {
        queue_.callAvailable ( ros::WallDuration ( timeout ) );
    }
}

void GazeboRosOmniDrive::UpdateOdometryEncoder()
{
    double m1 = joints_[JOINT1]->GetVelocity ( 0 );
    double m2 = joints_[JOINT2]->GetVelocity ( 0 );
    double m3 = joints_[JOINT3]->GetVelocity ( 0 );
#if GAZEBO_MAJOR_VERSION >= 8
    common::Time current_time = parent->GetWorld()->SimTime();
#else
    common::Time current_time = parent->GetWorld()->GetSimTime();
#endif
    double seconds_since_last_update = ( current_time - last_odom_update_ ).Double();
    last_odom_update_ = current_time;

    // double b = wheel_separation_;
    // // Book: Sigwart 2011 Autonompus Mobile Robots page:337
    // double sl = vl * ( wheel_diameter_ / 2.0 ) * seconds_since_last_update;
    // double sr = vr * ( wheel_diameter_ / 2.0 ) * seconds_since_last_update;
    // double ssum = sl + sr;

    // double sdiff = sr - sl;

    // double dx = ( ssum ) /2.0 * cos ( pose_encoder_.theta + ( sdiff ) / ( 2.0*b ) );
    // double dy = ( ssum ) /2.0 * sin ( pose_encoder_.theta + ( sdiff ) / ( 2.0*b ) );
    // double dtheta = ( sdiff ) /b;
    double vx = -1/3*( 0*m1 + m2*sqrt(3) - m3*sqrt(3));
    double vy = -1/3*(-2*m1 + m2 + m3);
    double vz = -1/3*(-1*m1 - m2 - m3);
    // double vx = 1/3*( 0*m1 + m2*sqrt(3) - m3*sqrt(3));
    // double vy = 1/3*(-2*m1 + m2 + m3);
    // double vz = 1/3*(-1*m1 - m2 - m3);

    double dy = (vy * cos(pose_encoder_.theta) - vx * sin(pose_encoder_.theta)) * ( wheel_diameter_ / 2.0 ) * seconds_since_last_update;
    double dx = (vy * sin(pose_encoder_.theta) + vx * cos(pose_encoder_.theta)) * ( wheel_diameter_ / 2.0 ) * seconds_since_last_update;
    double dtheta = vz * ( wheel_diameter_ / 2.0 ) * seconds_since_last_update / omni_radii_;

    pose_encoder_.x += dx;
    pose_encoder_.y += dy;
    pose_encoder_.theta += dtheta;

    double W = dtheta/seconds_since_last_update;
    double VX = sqrt ( dx*dx ) /seconds_since_last_update;
    double VY = sqrt ( dy*dy ) /seconds_since_last_update;

    tf::Quaternion qt;
    tf::Vector3 vt;
    qt.setRPY ( 0,0,pose_encoder_.theta );
    vt = tf::Vector3 ( pose_encoder_.x, pose_encoder_.y, 0 );

    odom_.pose.pose.position.x = vt.x();
    odom_.pose.pose.position.y = vt.y();
    odom_.pose.pose.position.z = vt.z();

    odom_.pose.pose.orientation.x = qt.x();
    odom_.pose.pose.orientation.y = qt.y();
    odom_.pose.pose.orientation.z = qt.z();
    odom_.pose.pose.orientation.w = qt.w();

    odom_.twist.twist.angular.z = W;
    odom_.twist.twist.linear.x  = VX;
    odom_.twist.twist.linear.y  = VY;
}

void GazeboRosOmniDrive::publishOdometry ( double step_time )
{

    ros::Time current_time = ros::Time::now();
    std::string odom_frame = gazebo_ros_->resolveTF ( odometry_frame_ );
    std::string base_footprint_frame = gazebo_ros_->resolveTF ( robot_base_frame_ );

    tf::Quaternion qt;
    tf::Vector3 vt;

    if ( odom_source_ == ENCODER ) {
        // getting data form encoder integration
        qt = tf::Quaternion ( odom_.pose.pose.orientation.x, odom_.pose.pose.orientation.y, odom_.pose.pose.orientation.z, odom_.pose.pose.orientation.w );
        vt = tf::Vector3 ( odom_.pose.pose.position.x, odom_.pose.pose.position.y, odom_.pose.pose.position.z );

    }
    if ( odom_source_ == WORLD ) {
        // getting data from gazebo world
#if GAZEBO_MAJOR_VERSION >= 8
        ignition::math::Pose3d pose = parent->WorldPose();
#else
        ignition::math::Pose3d pose = parent->GetWorldPose().Ign();
#endif
        qt = tf::Quaternion ( pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W() );
        vt = tf::Vector3 ( pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z() );

        odom_.pose.pose.position.x = vt.x();
        odom_.pose.pose.position.y = vt.y();
        odom_.pose.pose.position.z = vt.z();

        odom_.pose.pose.orientation.x = qt.x();
        odom_.pose.pose.orientation.y = qt.y();
        odom_.pose.pose.orientation.z = qt.z();
        odom_.pose.pose.orientation.w = qt.w();

        // get velocity in /odom frame
        ignition::math::Vector3d linear;
#if GAZEBO_MAJOR_VERSION >= 8
        linear = parent->WorldLinearVel();
        odom_.twist.twist.angular.z = parent->WorldAngularVel().Z();
#else
        linear = parent->GetWorldLinearVel().Ign();
        odom_.twist.twist.angular.z = parent->GetWorldAngularVel().Ign().Z();
#endif

        // convert velocity to child_frame_id (aka base_footprint)
        float yaw = pose.Rot().Yaw();
        odom_.twist.twist.linear.x = cosf ( yaw ) * linear.X() + sinf ( yaw ) * linear.Y();
        odom_.twist.twist.linear.y = cosf ( yaw ) * linear.Y() - sinf ( yaw ) * linear.X();
    }

    if (publishOdomTF_ == true){
        tf::Transform base_footprint_to_odom ( qt, vt );
        transform_broadcaster_->sendTransform (
            tf::StampedTransform ( base_footprint_to_odom, current_time,
                                   odom_frame, base_footprint_frame ) );
    }


    // set covariance
    odom_.pose.covariance[0] = 0.00001;
    odom_.pose.covariance[7] = 0.00001;
    odom_.pose.covariance[14] = 1000000000000.0;
    odom_.pose.covariance[21] = 1000000000000.0;
    odom_.pose.covariance[28] = 1000000000000.0;
    odom_.pose.covariance[35] = 0.001;


    // set header
    odom_.header.stamp = current_time;
    odom_.header.frame_id = odom_frame;
    odom_.child_frame_id = base_footprint_frame;

    odometry_publisher_.publish ( odom_ );
}

GZ_REGISTER_MODEL_PLUGIN ( GazeboRosOmniDrive )
}
