#include <wm_mecanum_base_controller/wm_mecanum_base_controller.h>
#include <tf/transform_datatypes.h>
#include <ros/console.h>
#include <string>
#include <iostream>
#include <ros/time.h>
#include <math.h>

#define PI 3.1415926535

namespace wm_mecanum_base_controller_ns
{
    WMMecanumBaseController::WMMecanumBaseController():
        command_struct_()
        ,dk_x_{0.0, 0.0, 0.0, 0.0}
        ,dk_y_{0.0, 0.0, 0.0, 0.0}
        ,dk_yaw_{0.0, 0.0, 0.0, 0.0}
        ,ik_{{0,0,0},{0,0,0},{0,0,0}}
        ,x_linear_speed_(0.0)
        ,y_linear_speed_(0.0)
        ,angular_speed_(0.0)
        ,lin_max_vel_(0.0)
        ,ang_max_vel_(0.0)
        ,robot_pose_x_(0.0)
        ,robot_pose_y_(0.0)
        ,robot_pose_orientation_x_(0.0)
        ,robot_pose_orientation_y_(0.0)
        ,robot_pose_orientation_z_(0.0)
        ,robot_pose_orientation_w_(0.0)
        ,x_wheel_to_center_(0.0)
        ,y_wheel_to_center_(0.0)
        ,wheel_radius_(0.0)
        ,gb_ratio_(0.0)
        ,cmd_vel_timeout_(0.5)
        ,base_frame_("base_link")
        ,read_state_(false)
        ,i_front_left_vel_(0)
        ,i_front_right_vel_(1)
        ,i_rear_left_vel_(2)
        ,i_rear_right_vel_(3)
        ,enable_odom_tf_(false)
        {}

    bool WMMecanumBaseController::init(hardware_interface::VelocityJointInterface* hw,
      ros::NodeHandle& root_nh,
      ros::NodeHandle &controller_nh)
    {
        const std::string complete_ns = controller_nh.getNamespace();
        std::size_t id = complete_ns.find_last_of("/");
        name_ = complete_ns.substr(id + 1);

        /// Get parameters
        // x axis distance between wheel axis and the robot's centroid
        controller_nh.param<double>("x_wheel_to_center", x_wheel_to_center_, 0.31 );
        ROS_INFO_STREAM_NAMED(name_, "The x distance between wheel and center is : " << x_wheel_to_center_);

        // y axis distance between wheel radial median and the robot'S centroid
        controller_nh.param<double>("y_wheel_to_center", y_wheel_to_center_, 0.30 );
        ROS_INFO_STREAM_NAMED(name_, "The Y distance between wheel and center is : " << y_wheel_to_center_);

        // Wheel radius
        controller_nh.param<double>("wheel_radius", wheel_radius_, 0.075 );
        ROS_INFO_STREAM_NAMED(name_, "The wheels radius is : " << wheel_radius_);

        // Gear box ration
        controller_nh.param<double>("gb_ratio_", gb_ratio_, 15.0);
        ROS_INFO_STREAM_NAMED(name_, "The gear box ratio is : " << gb_ratio_);

        // max linear velocity, in m/s
        controller_nh.param<double>("lin_max_vel", lin_max_vel_ , 1.0);
        ROS_INFO_STREAM_NAMED(name_, "The max linear velocity is : " << lin_max_vel_);

        // max angular velocity, in rad/s
        controller_nh.param<double>("ang_max_vel", ang_max_vel_, 1.0);
        ROS_INFO_STREAM_NAMED(name_, "The max angular velocity is : " << ang_max_vel_);

        // Get joint names from the parameter server
        controller_nh.param("front_left_wheel_joint", front_left_wheel_name_, front_left_wheel_name_);
        ROS_INFO_STREAM_NAMED(name_, "Front left wheel joint name is : " + front_left_wheel_name_);

        // Get joint names from the parameter server
        controller_nh.param("front_right_wheel_joint", front_right_wheel_name_, front_right_wheel_name_);
        ROS_INFO_STREAM_NAMED(name_, "Front right wheel joint name is : " + front_right_wheel_name_);

        // Get joint names from the parameter server
        controller_nh.param("rear_left_wheel_joint", rear_left_wheel_name_, rear_left_wheel_name_);
        ROS_INFO_STREAM_NAMED(name_, "rear left wheel joint name is : " + rear_left_wheel_name_);

        // Get joint names from the parameter server
        controller_nh.param("rear_right_wheel_joint", rear_right_wheel_name_, rear_right_wheel_name_);
        ROS_INFO_STREAM_NAMED(name_, "Front right wheel joint name is : " + rear_right_wheel_name_);

        // Odometry related:
        controller_nh.param("publish_rate", publish_rate_, 50.0);
        ROS_INFO_STREAM_NAMED(name_, "Controller state will be published at " + std::to_string(publish_rate_) + "Hz.");
        publish_period_ = ros::Duration(1.0 / publish_rate_);

        // Twist command related:
        controller_nh.param("cmd_vel_timeout", cmd_vel_timeout_, cmd_vel_timeout_);
        ROS_INFO_STREAM_NAMED(name_, "Velocity commands will be considered old if they are older than "
                              << cmd_vel_timeout_ << "s.");

        controller_nh.param("base_frame", base_frame_, base_frame_);
        ROS_INFO_STREAM_NAMED(name_, "Base frame_id set to " + base_frame_);

        controller_nh.param("enable_odom_tf", enable_odom_tf_, enable_odom_tf_);
        ROS_INFO_STREAM_NAMED(name_, "Publishing to tf is " << (enable_odom_tf_?"enabled":"disabled"));

        // Initialize inverce and direct kinematic matrix with param values
        for(int i = 0; i < 4; i++)
        {
            dk_yaw_[i] = -1.0 / 4 * (x_wheel_to_center_ + y_wheel_to_center_);

            ik_[i][0] = 1;
            ik_[i][1] = (i==0||3) ? -1 : 1;
            ik_[i][2] = -((-1)^i)*(x_wheel_to_center_ + y_wheel_to_center_);
        }

        // Get handle of the joint
        front_left_wheel_joint_    = hw->getHandle(front_left_wheel_name_);  // throws on failure
        front_right_wheel_joint_    = hw->getHandle(front_right_wheel_name_);  // throws on failure
        rear_left_wheel_joint_    = hw->getHandle(rear_left_wheel_name_);  // throws on failure
        rear_right_wheel_joint_    = hw->getHandle(rear_right_wheel_name_);  // throws on failure

        // Start realtime state publisher
        // controller_state_publisher_.reset(
        // new realtime_tools::RealtimePublisher<control_msgs::JointControllerState>(controller_nh, "state", 1));

        setOdomPubFields(root_nh, controller_nh);

        sub_command_ = controller_nh.subscribe("cmd_vel", 1, &WMMecanumBaseController::cmdVelCallback, this);
        sub_joint_state_ = controller_nh.subscribe<sensor_msgs::JointState>("/joint_states", 1, &WMMecanumBaseController::jointStateCallback, this);

        // Initialize joint indexes according to joint names
        if (read_state_)
        {
            std::vector<std::string> joint_names = joint_state_.name;
            i_front_left_vel_ = find(joint_names.begin(), joint_names.end(), std::string(front_left_wheel_name_)) - joint_names.begin();
            i_front_right_vel_= find(joint_names.begin(), joint_names.end(), std::string(front_right_wheel_name_)) - joint_names.begin();
            i_rear_left_vel_  = find(joint_names.begin(), joint_names.end(), std::string(rear_left_wheel_name_)) - joint_names.begin();
            i_rear_right_vel_ = find(joint_names.begin(), joint_names.end(),std:: string(rear_right_wheel_name_)) - joint_names.begin();
            return 0;
        }
        else
        {
    	  ROS_INFO("Joint State not received");
    	  return -1;
        }

        return true;
    }

    void WMMecanumBaseController::update(const ros::Time& time, const ros::Duration& period)
    {
        // Update and Publish odometry message
        if (last_state_publish_time_ + publish_period_ < time)
        {
            last_state_publish_time_ += publish_period_;
            updateOdometry(time);
            publishOdometry(time);
        }

        // MOVE ROBOT
        // Retreive current velocity command and time step:
        Commands curr_cmd = *(command_.readFromRT());
        const double dt = (time - curr_cmd.stamp).toSec();
        double W[4] = {0.0, 0.0, 0.0, 0.0};
        double v;

        // Brake if cmd_vel has timeout:
        if (dt > cmd_vel_timeout_)
        {
            brake();
        }

        // Limit velocities and accelerations:
        const double cmd_dt(period.toSec());

        v = sqrt(pow(curr_cmd.lin.x,2) + pow(curr_cmd.lin.y,2));
        last1_v_ = last0_v_;
        last0_v_ = v;

        limiter_lin_.limit(v, last0_v_, last1_v_, cmd_dt);
        limiter_ang_.limit(curr_cmd.ang.z, last0_cmd_.ang.z, last1_cmd_.ang.z, cmd_dt);

        last1_cmd_ = last0_cmd_;
        last0_cmd_ = curr_cmd;

        InverseKinematics(&curr_cmd, W);

        front_left_wheel_joint_.setCommand(W[0]);
        front_right_wheel_joint_.setCommand(W[1]);
        rear_left_wheel_joint_.setCommand(W[2]);
        rear_right_wheel_joint_.setCommand(W[3]);
    }

    void WMMecanumBaseController::starting(const ros::Time& time)
    {
        brake();

        // Register starting time used to keep fixed rate
        last_state_publish_time_ = time;
    }

    void WMMecanumBaseController::stopping(const ros::Time& /*time*/)
    {
        brake();
    }

    void WMMecanumBaseController::InverseKinematics(struct Commands * cmd, double W[4])
    {
        // Reference:
        // Maulana, E.; Muslim, M.A.; Hendrayawan, V.,
        // "Inverse kinematic implementation of four-wheels mecanum drive mobile robot using stepper motors,"
        //  in Intelligent Technology and Its Applications (ISITIA), 2015 International Seminar on ,
        // vol., no., pp.51-56, 20-21 May 2015

        // linear velocity
        double vLinear = sqrt(pow(cmd->lin.x,2) + pow(cmd->lin.y,2));

        if(vLinear > lin_max_vel_)
        {
            vLinear = lin_max_vel_;
        }

        // movement orientation
        double Heading = atan2(cmd->lin.y, cmd->lin.x);

        // x axis linear velocity
        double x_vel = vLinear * cos(Heading);
        // y axis linear velocity
        double y_vel = vLinear * sin(Heading);

        // YAW axis rotational velocity
        double ang_vel = cmd->ang.z;

        if(pow(ang_vel,2) > pow(ang_max_vel_,2))
        {
            ang_vel =  ang_max_vel_ * ang_vel / abs(ang_vel);
        }

        // x y and angular velocity
        double V[3] = {x_vel, y_vel, ang_vel};

        // matrix multiplaction
        for(int i = 0; i < 3; i++)
        {
            for(int k = 0; k < 4; k++)
            {
                W[i] += ik_[i][k] * V[k];
            }
            W[i] = W[i] * 1/gb_ratio_ * 1/wheel_radius_;
        }
    }

    void WMMecanumBaseController::brake()
    {
        front_left_wheel_joint_.setCommand(0.0);
        front_right_wheel_joint_.setCommand(0.0);
        front_left_wheel_joint_.setCommand(0.0);
        front_right_wheel_joint_.setCommand(0.0);
    }

    // TODO update Odometry
    void WMMecanumBaseController::updateOdometry(const ros::Time& time)
    {

        if(read_state_)
        {
            const double dt = (time - last_state_publish_time_).toSec();
            last_state_publish_time_ = time;

            // x axis linear velocity
            // multiply by -1.0 because of wiring
            x_linear_speed_ = -1.0 / gb_ratio_ * wheel_radius_ * (dk_x_[0]*joint_state_.velocity[i_front_left_vel_] +
                                                                  dk_x_[1]*joint_state_.velocity[i_front_right_vel_] +
                                                                  dk_x_[2]*joint_state_.velocity[i_rear_left_vel_] +
                                                                  dk_x_[3]*joint_state_.velocity[i_rear_right_vel_]);

            // y axis linear velocity
            // multiply by -1.0 because of wiring
            y_linear_speed_ = -1.0 / gb_ratio_ * wheel_radius_ * (dk_y_[0]*joint_state_.velocity[i_front_left_vel_] +
                                                                  dk_y_[1]*joint_state_.velocity[i_front_right_vel_] +
                                                                  dk_y_[2]*joint_state_.velocity[i_rear_left_vel_] +
                                                                  dk_y_[3]*joint_state_.velocity[i_rear_right_vel_]);

            // yaw angular velocity
            // multiply by -1.0 because of wiring
            angular_speed_ = -1.0 / gb_ratio_ * wheel_radius_ * (dk_yaw_[0]*joint_state_.velocity[i_front_left_vel_] +
                                                                 dk_yaw_[1]*joint_state_.velocity[i_front_right_vel_] +
                                                                 dk_yaw_[2]*joint_state_.velocity[i_rear_left_vel_] +
                                                                 dk_yaw_[3]*joint_state_.velocity[i_rear_right_vel_]);

            // update pose orientation
            // convert orientation to RPY
            // euler = [roll, pitch, yaw]
            double euler[3];
            tf::Quaternion q(robot_pose_orientation_x_, robot_pose_orientation_y_, robot_pose_orientation_z_, robot_pose_orientation_w_);
            tf::Matrix3x3 m(q);
            m.getRPY(euler[0], euler[1], euler[2]);


            robot_heading_ = euler[2];

            // update x position
            robot_pose_x_ += dt * (x_linear_speed_ * cos(euler[2]) - y_linear_speed_ * sin(euler[2]));
            // update y position
            robot_pose_y_ += dt * (x_linear_speed_ * sin(euler[2]) + y_linear_speed_ * cos(euler[2]));

            // add yaw variation to RPY and convert orientation back to quaternion
            // Assuming a flat world, only roatation around Z
            q = tf::createQuaternionFromRPY(0.0, 0.0, euler[2]);

            robot_pose_orientation_x_ = q.getX();
            robot_pose_orientation_y_ = q.getY();
            robot_pose_orientation_z_ = q.getZ();
            robot_pose_orientation_w_ = q.getZ();
        }
    }

    void WMMecanumBaseController::publishOdometry(const ros::Time& time)
    {
       // Populate odom message and publish
        if (odom_pub_->trylock())
        {
            // Compute and store orientation info
            const geometry_msgs::Quaternion orientation(tf::createQuaternionMsgFromYaw(robot_heading_));

              odom_pub_->msg_.header.stamp = time;
	          odom_pub_->msg_.pose.pose.position.x = robot_pose_x_;
	          odom_pub_->msg_.pose.pose.position.y =robot_pose_y_;
	          odom_pub_->msg_.pose.pose.orientation = orientation;
              odom_pub_->msg_.twist.twist.linear.x  = x_linear_speed_;
              odom_pub_->msg_.twist.twist.linear.y  = y_linear_speed_;
              odom_pub_->msg_.twist.twist.angular.z = angular_speed_;
              odom_pub_->unlockAndPublish();
        }

        // Publish tf /odom frame
        if (enable_odom_tf_ && tf_odom_pub_->trylock())
        {
            // Compute and store orientation info
            const geometry_msgs::Quaternion orientation(tf::createQuaternionMsgFromYaw(robot_heading_));

            geometry_msgs::TransformStamped& odom_frame = tf_odom_pub_->msg_.transforms[0];
            odom_frame.header.stamp = time;
            odom_frame.transform.translation.x = robot_pose_x_;
            odom_frame.transform.translation.y = robot_pose_y_;
            odom_frame.transform.rotation = orientation;
            tf_odom_pub_->unlockAndPublish();
        }
    }

    void WMMecanumBaseController::jointStateCallback(const sensor_msgs::JointStateConstPtr &msg)
    {
        joint_state_ = *msg;
        read_state_ = true;
    }

    void WMMecanumBaseController::cmdVelCallback(const geometry_msgs::Twist& command)
    {
        command_struct_.ang.z   = command.angular.z;
        command_struct_.lin.x   = command.linear.x;
        command_struct_.stamp = ros::Time::now();
        command_.writeFromNonRT (command_struct_);

        ROS_DEBUG_STREAM_NAMED(name_,
                               "Added values to command. "
                               << "Ang: "   << command_struct_.ang.z << ", "
                               << "Lin: "   << command_struct_.lin.x << ", "
                               << "Stamp: " << command_struct_.stamp.toSec());
    }

    void WMMecanumBaseController::setOdomPubFields(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
    {
        // Get and check params for covariances
        XmlRpc::XmlRpcValue pose_cov_list;
        controller_nh.getParam("pose_covariance_diagonal", pose_cov_list);
        ROS_ASSERT(pose_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
        ROS_ASSERT(pose_cov_list.size() == 6);
        for (int i = 0; i < pose_cov_list.size(); ++i)
            ROS_ASSERT(pose_cov_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

        XmlRpc::XmlRpcValue twist_cov_list;
        controller_nh.getParam("twist_covariance_diagonal", twist_cov_list);
        ROS_ASSERT(twist_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
        ROS_ASSERT(twist_cov_list.size() == 6);
        for (int i = 0; i < twist_cov_list.size(); ++i)
            ROS_ASSERT(twist_cov_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

        // Setup odometry realtime publisher + odom message constant fields
        odom_pub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(controller_nh, "odom", 100));
        odom_pub_->msg_.header.frame_id = "odom";
        odom_pub_->msg_.child_frame_id = base_frame_;
        odom_pub_->msg_.pose.pose.position.z = 0;
        odom_pub_->msg_.pose.covariance = boost::assign::list_of
                (static_cast<double>(pose_cov_list[0])) (0)  (0)  (0)  (0)  (0)
                (0)  (static_cast<double>(pose_cov_list[1])) (0)  (0)  (0)  (0)
                (0)  (0)  (static_cast<double>(pose_cov_list[2])) (0)  (0)  (0)
                (0)  (0)  (0)  (static_cast<double>(pose_cov_list[3])) (0)  (0)
                (0)  (0)  (0)  (0)  (static_cast<double>(pose_cov_list[4])) (0)
                (0)  (0)  (0)  (0)  (0)  (static_cast<double>(pose_cov_list[5]));
        odom_pub_->msg_.twist.twist.linear.y  = 0;
        odom_pub_->msg_.twist.twist.linear.z  = 0;
        odom_pub_->msg_.twist.twist.angular.x = 0;
        odom_pub_->msg_.twist.twist.angular.y = 0;
        odom_pub_->msg_.twist.covariance = boost::assign::list_of
                (static_cast<double>(twist_cov_list[0])) (0)  (0)  (0)  (0)  (0)
                (0)  (static_cast<double>(twist_cov_list[1])) (0)  (0)  (0)  (0)
                (0)  (0)  (static_cast<double>(twist_cov_list[2])) (0)  (0)  (0)
                (0)  (0)  (0)  (static_cast<double>(twist_cov_list[3])) (0)  (0)
                (0)  (0)  (0)  (0)  (static_cast<double>(twist_cov_list[4])) (0)
                (0)  (0)  (0)  (0)  (0)  (static_cast<double>(twist_cov_list[5]));
        tf_odom_pub_.reset(new realtime_tools::RealtimePublisher<tf::tfMessage>(root_nh, "/tf", 100));
        tf_odom_pub_->msg_.transforms.resize(1);
        tf_odom_pub_->msg_.transforms[0].transform.translation.z = 0.0;
        tf_odom_pub_->msg_.transforms[0].child_frame_id = base_frame_;
        tf_odom_pub_->msg_.transforms[0].header.frame_id = "odom";
    }

} // namespace wm_mecanum_base_controller_ns
