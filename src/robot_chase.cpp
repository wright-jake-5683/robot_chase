#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/exceptions.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include "std_msgs/msg/string.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

struct Coordinates
{
  double x, y, z;
  double roll, pitch, yaw;

  Coordinates(double x, double y, double z, double roll, double pitch, double yaw)
    : x(x), y(y), z(z), roll(roll), pitch(pitch), yaw(yaw) {}
};

class RobotChase : public rclcpp::Node
{
    public:
        RobotChase(double timer_period=0.05, double trans_speed = 1.0, double rot_speed = 0.1, const std::string &init_dest_frame = "morty/base_link")
            : Node("robot_chase_node"), timer_period_(timer_period), trans_speed_(trans_speed), rot_speed_(rot_speed)
        {
            set_desination_frame(init_dest_frame);

            // Set up TF listener
            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);

            timer_ = this->create_wall_timer(std::chrono::duration<double>(timer_period_), std::bind(&RobotChase::timer_callback, this));

            auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
            dest_frame_sub_ = this->create_subscription<std_msgs::msg::String>(
                "/destination_frame", 
                qos, 
                std::bind(&RobotChase::move_callback, this, std::placeholders::_1)
            );

            cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
                "rick/cmd_vel", 
                10
            );
        }


        void set_translation_speed(double speed) { trans_speed_ = speed; }
        void set_rotation_speed(double speed)    { rot_speed_   = speed; }

        // sets code up to allow rick to follow any frame set to objective_frame_
        void set_desination_frame(const std::string &new_dest_frame)
        {
            objective_frame_ = new_dest_frame;
        }  


    private:
 // ── Callbacks ────────────────────────────────────────────────────────────
        void timer_callback()
        {
            find_coordinates_and_chase();
        }

        void move_callback(const std_msgs::msg::String::SharedPtr msg)
        {
            set_desination_frame(msg->data);
        }


// ── Motion helpers ───────────────────────────────────────────────────────
        void find_coordinates_and_chase()
        {
            std::shared_ptr<Coordinates> chaser_coords = calculate_coord("rick/base_link");
            std::shared_ptr<Coordinates> runner_coords = calculate_coord(objective_frame_);

            //RCLCPP_INFO(this->get_logger(), "chaser coords-> x:%.2f, y:%.2f", chaser_coords->x, chaser_coords->y);
            //RCLCPP_INFO(this->get_logger(), "runner coords-> x:%.2f, y:%.2f", runner_coords->x, runner_coords->y);

            chase(chaser_coords, runner_coords);
        }
        
        std::shared_ptr<geometry_msgs::msg::Pose> get_model_pose_from_tf (const std::string &dest_frame, const std::string &origin_frame = "world")
        {
            geometry_msgs::msg::TransformStamped transform;

            try
            {
                transform = tf_buffer_->lookupTransform(origin_frame.c_str(), dest_frame.c_str(), tf2::TimePointZero, tf2::durationFromSec(1.0));
            }
            catch (const tf2::TransformException &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Could not find transfrom from %s to %s. \nException: %s", origin_frame.c_str(), dest_frame.c_str(), e.what());
                return nullptr;
            }

            auto &translation = transform.transform.translation;
            auto &rotation = transform.transform.rotation;

            //RCLCPP_INFO(this->get_logger(), "translation: [%f, %f, %f]", translation.x, translation.y, translation.z);
            //RCLCPP_INFO(this->get_logger(), "rotation: [%f, %f, %f, %f]", rotation.x, rotation.y, rotation.z, rotation.w);

            auto pose = std::make_shared<geometry_msgs::msg::Pose>();
            pose->position.x = translation.x;
            pose->position.y = translation.y;
            pose->position.z = translation.z;
            pose->orientation.x = rotation.x;
            pose->orientation.y = rotation.y;
            pose->orientation.z = rotation.z;
            pose->orientation.w = rotation.w;

            return pose;
        }

        // converts Transforms orientation to quaternion to extract roll, pitch, and yaw
        std::shared_ptr<Coordinates> calculate_coord(const std::string &dest_frame)
        {
            auto pose = get_model_pose_from_tf(dest_frame);

            if (!pose) return nullptr;

            // --- Convert orientations ---
            tf2::Quaternion target_q(
                pose->orientation.x,
                pose->orientation.y,
                pose->orientation.z,
                pose->orientation.w
            );
            
            double roll, pitch, yaw;
            tf2::Matrix3x3(target_q).getRPY(roll, pitch, yaw);


            return std::make_shared<Coordinates>(
                pose->position.x, 
                pose->position.y, 
                pose->position.z,
                roll, 
                pitch, 
                yaw
            );
        }

        void chase(std::shared_ptr<Coordinates> chaser_coords, std::shared_ptr<Coordinates> runner_coords)
        {
             // --- Position error ---
            double dx = runner_coords->x - chaser_coords->x;
            double dy = runner_coords->y - chaser_coords->y;
            double distance = std::sqrt(dx*dx + dy*dy);
            //RCLCPP_INFO(this->get_logger(), "distance: %.2f", distance);


            // --- Angle to target position ---
            double dyaw = std::atan2(dy, dx);


            // --- Heading error, normalized to [-pi, pi] ---
            double yaw_error = dyaw - chaser_coords->yaw;
            while (yaw_error >  M_PI) yaw_error -= 2.0 * M_PI;
            while (yaw_error < -M_PI) yaw_error += 2.0 * M_PI;

            //RCLCPP_INFO(this->get_logger(), "dyaw: %.2f", dyaw);
            //RCLCPP_INFO(this->get_logger(), "chaser yaw: %.2f", chaser_coords->yaw);
            //RCLCPP_INFO(this->get_logger(), "yaw_error: %.2f", yaw_error);

            //constexpr tells complier kp_yaw and kp_distance values at complie time instead of at runtime
            constexpr double kp_yaw = 1;
            double kp_distance;
            if (yaw_error > -0.1 && yaw_error < 0.1)
            {
                kp_distance = 0.75;
            }
            else 
            {
                kp_distance = 0.45;
            }

            geometry_msgs::msg::Twist cmd;

            RCLCPP_INFO(this->get_logger(), "kp_distance=%.2f", kp_distance);
            cmd.linear.x = kp_distance * distance;
            cmd.angular.z = kp_yaw * yaw_error;


            // --- Stop if close enough ---
            if (distance < 0.37) {
                cmd.linear.x  = 0.0;
                cmd.angular.z = 0.0;
            }

            cmd_vel_pub_->publish(cmd);
        }


// ── Members ──────────────────────────────────────────────────────────────    
        std::string objective_frame_;
        double trans_speed_;
        double rot_speed_;
        double timer_period_;

        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        rclcpp::TimerBase::SharedPtr timer_;

        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr dest_frame_sub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

};


// ── main ─────────────────────────────────────────────────────────────────────
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<RobotChase>();

  std::cout << "Start Chase" << std::endl;
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
