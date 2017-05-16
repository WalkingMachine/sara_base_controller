#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <urdf_parser/urdf_parser.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>
#include <boost/function.hpp>
#include <boost/assign.hpp>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <tf/tfMessage.h>
#include <wm_mecanum_base_controller/speed_limiter.h>

namespace wm_mecanum_base_controller_ns
{
  /**
   * This class makes some assumptions on the model of the robot:
   *  - the rotation axes of wheels are collinear
   *  - the wheels are identical in radius
   */
    class WMMecanumBaseController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
    {
        public:
          WMMecanumBaseController();

          /**
           * \brief Initialize controller
           * \param hw            Velocity joint interface for the wheels
           * \param root_nh       Node handle at root namespace
           * \param controller_nh Node handle inside the controller namespace
           */
            bool init(hardware_interface::VelocityJointInterface* hw,
              ros::NodeHandle& root_nh,
              ros::NodeHandle &controller_nh);

            /**
             * \brief Updates controller, i.e. computes the odometry and sets the new velocity commands
             * \param time   Current time
             * \param period Time since the last called to update
             */
            void update(const ros::Time& time, const ros::Duration& period);

            /**
             * \brief Starts controller
             * \param time Current time
             */
            void starting(const ros::Time& time);

            /**
             * \brief Stops controller
             * \param time Current time
             */
            void stopping(const ros::Time& time);

        private:
            std::string name_;

            // Robot Joint States
            sensor_msgs::JointState joint_state_;

            /// Odometry related:
            ros::Duration publish_period_;
            ros::Time last_state_publish_time_;
            double publish_rate_;

            // Velocity command related:
            struct Commands
            {
              urdf::Vector3 lin;
              urdf::Vector3 ang;
              ros::Time stamp;

              Commands() : lin(0.0,0.0,0.0), ang(0.0,0.0,0.0), stamp(0.0) {}
            };

            // Joint names
            std::string front_left_wheel_name_;
            std::string front_right_wheel_name_;
            std::string rear_left_wheel_name_;
            std::string rear_right_wheel_name_;

            // Hardware handles:
            hardware_interface::JointHandle front_right_wheel_joint_;
            hardware_interface::JointHandle front_left_wheel_joint_;
            hardware_interface::JointHandle rear_right_wheel_joint_;
            hardware_interface::JointHandle rear_left_wheel_joint_;

            // realtime stuff
            //boost::scoped_ptr<realtime_tools::RealtimePublisher< control_msgs::JointControllerState> > controller_state_publisher_ ;
            realtime_tools::RealtimeBuffer<Commands> command_;
            Commands command_struct_;

            /// Subscriber
            ros::Subscriber sub_command_;
            ros::Subscriber sub_joint_state_;

            /// Variables
            // Odometry related:
            boost::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry> > odom_pub_;
            boost::shared_ptr<realtime_tools::RealtimePublisher<tf::tfMessage> > tf_odom_pub_;
            double dk_x_[4] = {1.0/4, -1.0/4, 1.0/4, -1.0/4};
            double dk_y_[4] = {-1.0/4, -1.0/4, 1.0/4, 1.0/4};
            double dk_yaw_[4];
            double ik_[4][3];

            // Wheel separation
            double x_wheel_to_center_;
            double y_wheel_to_center_;
            double front_dist_to_center_;
            double rear_dist_to_center_;

            // Robot position
            double robot_pose_x_;
            double robot_pose_y_;
            double robot_heading_;
            double robot_pose_orientation_x_;
            double robot_pose_orientation_y_;
            double robot_pose_orientation_z_;
            double robot_pose_orientation_w_;

            // Wheel radius:
            double wheel_radius_;

            // gear box ration
            double gb_ratio_;

            // Timeout to consider cmd_vel commands old:
            double cmd_vel_timeout_;

            // Frame to use for the robot base:
            std::string base_frame_;

            // Whether to publish odometry to tf or not:
            bool enable_odom_tf_;

            // Speed limiters:
            Commands last1_cmd_;
            Commands last0_cmd_;
            double last1_v_;
            double last0_v_;
            SpeedLimiter limiter_lin_;
            SpeedLimiter limiter_ang_;

            // Speeds
            double x_linear_speed_;
            double y_linear_speed_;
            double angular_speed_;
            double lin_max_vel_;
            double ang_max_vel_;

            // Flag to indicate if joint_state_ has been read
            bool read_state_;

            // Indice for each wheel
            int i_front_left_vel_;
            int i_front_right_vel_;
            int i_rear_left_vel_;
            int i_rear_right_vel_;

            /// Methods
            /**
             * \brief Brakes the wheels, i.e. sets the velocity to 0
             * \param cmd last command received
             * \param W[] array to return velocity of all joints
             */
            void InverseKinematics(struct Commands * cmd, double W[4]);

            /**
            * \brief Brakes the wheels, i.e. sets the velocity to 0
            */
            void brake();

            /**
             * \brief Callback function for Joint_states
             * \param msg the joint state message
             */
            void jointStateCallback(const sensor_msgs::JointStateConstPtr &msg);

            /**
             * \brief Velocity command callback
             * \param command Velocity command message (twist)
             */
            void cmdVelCallback(const geometry_msgs::Twist& command);

            /**
             * \brief Publish the odometry
             */
            void publishOdometry(const ros::Time& time);

            /**
             * \brief Update the odometry based on the velocity of each wheel and the vehicule steering angle.
             */
            void updateOdometry(const ros::Time& time);

            /**
             * \brief Sets the odometry publishing fields
             * \param root_nh Root node handle
             * \param controller_nh Node handle inside the controller namespace
             */
            void setOdomPubFields(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);

    };
    PLUGINLIB_EXPORT_CLASS(wm_mecanum_base_controller_ns::WMMecanumBaseController, controller_interface::ControllerBase);
}
