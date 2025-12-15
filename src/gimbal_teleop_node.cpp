#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

class GimbalTeleopNode : public rclcpp::Node
{
    public:
        GimbalTeleopNode()
        : Node("gimbal_teleop_node"),
          target_yaw_(0.0),
          target_pitch_(0.0),
          step_rad_(0.02)
        {
            traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
                "/gimbal_controller/joint_trajectory", 10
            );

            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(50),
                std::bind(&GimbalTeleopNode::timerCallback, this)
            );

            enableRawMode();

        }
        ~GimbalTeleopNode(){
            disableRawMode();
        }
    private:
        void enableRawMode(){
            tcgetattr(STDIN_FILENO, &orig_termios_);
            termios raw = orig_termios_;
            raw.c_lflag &= ~(ICANON | ECHO);
            tcsetattr(STDIN_FILENO, TCSANOW, &raw);

            int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
            fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
        }

        void disableRawMode(){
            tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios_);
        }

        int readkey(){
            char c;
            int n = read(STDIN_FILENO, &c, 1);
            if(n == 1) return static_cast<int>(c);
            return -1;
        }

        void handleKey(int key){
            switch(key){
                case 'w':
                case 'W':
                    target_pitch_ -= step_rad_;
                    break;
                case 's':
                case 'S':
                    target_pitch_ += step_rad_;
                    break;
                case 'a':
                case 'A':
                    target_yaw_ += step_rad_;
                    break;
                case 'd':
                case 'D':
                    target_yaw_ -= step_rad_;
                    break;
                case 'r':
                case 'R':
                    target_pitch_ = 0;
                    target_yaw_ = 0;
                    break;
                case 'q':
                case 'Q':
                    rclcpp::shutdown();
                    break;
                default:
                    break;
            }
        }

        void publishTrajectory(){
            trajectory_msgs::msg::JointTrajectory traj;
            traj.joint_names = {"yaw_joint", "pitch_joint"};

            trajectory_msgs::msg::JointTrajectoryPoint p;
            p.positions = {target_yaw_, target_pitch_};
            p.time_from_start.sec = 0;
            p.time_from_start.nanosec = 100000000;

            traj.points.push_back(p);
            traj_pub_->publish(traj);
        }
        void timerCallback(){
            int key = readkey();
            if(key != -1){
                handleKey(key);
            }
            publishTrajectory();
        }


        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
        rclcpp::TimerBase::SharedPtr timer_;

        termios orig_termios_;
        double target_yaw_;
        double target_pitch_;
        double step_rad_;
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GimbalTeleopNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
