#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>

#include <thread>
#include <chrono>
#include <vector>
#include <cmath>
#include <mutex>

using namespace std::chrono_literals;

class MoveItToJointCommand : public rclcpp::Node
{
public:
    MoveItToJointCommand() : Node("moveit_to_jointcommand")
    {
        joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/joint_command", 10);

        joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&MoveItToJointCommand::jointStatesCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "/joint_command bridge started.");
        RCLCPP_INFO(this->get_logger(), "Subscribed to /joint_states");
    }

    void initialize()
    {
        RCLCPP_INFO(this->get_logger(), "start");

        RCLCPP_INFO(this->get_logger(), "Waiting for joint_states...");
        int wait_count = 0;
        while (!has_joint_states_ && rclcpp::ok())
        {
            std::this_thread::sleep_for(100ms);
            wait_count++;
            if (wait_count % 10 == 0)
            {
                RCLCPP_INFO(this->get_logger(), "Still waiting for joint_states... (%d seconds)", wait_count / 10);
            }
        }
        RCLCPP_INFO(this->get_logger(), "Joint states received!");

        printCurrentJointStates();

        RCLCPP_INFO(this->get_logger(), "Initializing MoveIt...");
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "panda_arm");

        move_group_->setPlanningTime(20.0);
        move_group_->setMaxVelocityScalingFactor(0.5);
        move_group_->setMaxAccelerationScalingFactor(0.5);
        move_group_->setNumPlanningAttempts(10);
        
        move_group_->setGoalPositionTolerance(0.01);
        move_group_->setGoalOrientationTolerance(0.1);
        move_group_->setGoalJointTolerance(0.01);
        
        RCLCPP_INFO(this->get_logger(), "MoveIt initialized");

        std::this_thread::sleep_for(1s);
        updateRobotStateFromJointStates();
        std::this_thread::sleep_for(500ms);

        RCLCPP_INFO(this->get_logger(), "INITIALIZATION COMPLETE");
        
        auto current_pose = move_group_->getCurrentPose().pose;
        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "Current end-effector pose:");
        RCLCPP_INFO(this->get_logger(), "  Position: (%.3f, %.3f, %.3f)",
                   current_pose.position.x, current_pose.position.y, current_pose.position.z);

        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "Starting pick and place sequence...");
        runPickAndPlace();
    }

private:
    double current_gripper_ = 0.04;
    bool has_joint_states_ = false;
    std::mutex joint_mutex_;
    std::vector<double> current_joint_positions_;
    std::vector<std::string> joint_names_;

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;

    void jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(joint_mutex_);
        
        current_joint_positions_.clear();
        joint_names_.clear();
        
        for (size_t i = 0; i < msg->name.size(); ++i)
        {
            const auto& name = msg->name[i];
            if (name.find("panda_joint") != std::string::npos && 
                name.length() == 12)
            {
                joint_names_.push_back(name);
                current_joint_positions_.push_back(msg->position[i]);
            }
        }
        
        if (current_joint_positions_.size() == 7)
        {
            has_joint_states_ = true;
        }
    }

    void printCurrentJointStates()
    {
        std::lock_guard<std::mutex> lock(joint_mutex_);
        if (current_joint_positions_.size() == 7)
        {
            RCLCPP_INFO(this->get_logger(), 
                       "Current joint_states: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                       current_joint_positions_[0], current_joint_positions_[1],
                       current_joint_positions_[2], current_joint_positions_[3],
                       current_joint_positions_[4], current_joint_positions_[5],
                       current_joint_positions_[6]);
        }
    }

    void updateRobotStateFromJointStates()
    {
        std::lock_guard<std::mutex> lock(joint_mutex_);
        
        if (current_joint_positions_.size() != 7)
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid joint positions size: %zu (expected 7)", 
                        current_joint_positions_.size());
            return;
        }

        moveit::core::RobotStatePtr current_state = move_group_->getCurrentState(3.0);
        if (!current_state)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to get current robot state from MoveIt");
            return;
        }
        
        current_state->setJointGroupPositions("panda_arm", current_joint_positions_);
        current_state->update();
        move_group_->setStartState(*current_state);
    }

    void runPickAndPlace()
    {
        tf2::Quaternion q;
        q.setRPY(M_PI, 0, 0);  // 180도 뒤집기

        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "━━━━━━━━━━━━━━━━━━━━━━━━━━");
        RCLCPP_INFO(this->get_logger(), "  첫 번째 물체 Pick & Place");
        RCLCPP_INFO(this->get_logger(), "━━━━━━━━━━━━━━━━━━━━━━━━━━");

        // 첫 번째 물체 - 현재 위치 근처
        geometry_msgs::msg::Pose pick_pose1;
        pick_pose1.position.x = 0.4;
        pick_pose1.position.y = 0.3;
        pick_pose1.position.z = 1.3;
        pick_pose1.orientation = tf2::toMsg(q);

        geometry_msgs::msg::Pose pick_approach1 = pick_pose1;
        pick_approach1.position.z += 0.1;

        geometry_msgs::msg::Pose place_pose1;
        place_pose1.position.x = 0.4;
        place_pose1.position.y = -0.3;
        place_pose1.position.z = 1.3;
        place_pose1.orientation = tf2::toMsg(q);

        geometry_msgs::msg::Pose place_approach1 = place_pose1;
        place_approach1.position.z += 0.1;

        // 베이스 회전 + 고정 방식으로 실행
        auto j1 = run_sequence(pick_approach1, "Pick 접근");
        if (!j1.empty())
        {
            auto j2 = run_sequence(pick_pose1, "Pick 하강");
            if (!j2.empty())
            {
                close_gripper(j2);
                run_sequence(pick_approach1, "Pick 상승");
                run_sequence(place_approach1, "Place 접근");
                auto j5 = run_sequence(place_pose1, "Place 하강");
                if (!j5.empty())
                {
                    open_gripper(j5);
                    run_sequence(place_approach1, "Place 상승");
                }
            }
        }

        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "━━━━━━━━━━━━━━━━━━━━━━━━━━");
        RCLCPP_INFO(this->get_logger(), "  두 번째 물체 Pick & Place");
        RCLCPP_INFO(this->get_logger(), "━━━━━━━━━━━━━━━━━━━━━━━━━━");

        geometry_msgs::msg::Pose pick_pose2;
        pick_pose2.position.x = 0.5;
        pick_pose2.position.y = 0.2;
        pick_pose2.position.z = 1.4;
        pick_pose2.orientation = tf2::toMsg(q);

        geometry_msgs::msg::Pose pick_approach2 = pick_pose2;
        pick_approach2.position.z += 0.1;

        geometry_msgs::msg::Pose place_pose2;
        place_pose2.position.x = 0.5;
        place_pose2.position.y = -0.2;
        place_pose2.position.z = 1.4;
        place_pose2.orientation = tf2::toMsg(q);

        geometry_msgs::msg::Pose place_approach2 = place_pose2;
        place_approach2.position.z += 0.1;

        auto j6 = run_sequence(pick_approach2, "Pick 접근");
        if (!j6.empty())
        {
            auto j7 = run_sequence(pick_pose2, "Pick 하강");
            if (!j7.empty())
            {
                close_gripper(j7);
                run_sequence(pick_approach2, "Pick 상승");
                run_sequence(place_approach2, "Place 접근");
                auto j10 = run_sequence(place_pose2, "Place 하강");
                if (!j10.empty())
                {
                    open_gripper(j10);
                    run_sequence(place_approach2, "Place 상승");
                }
            }
        }

        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "━━━━━━━━━━━━━━━━━━━━━━━━━━");
        RCLCPP_INFO(this->get_logger(), "완료");
        RCLCPP_INFO(this->get_logger(), "━━━━━━━━━━━━━━━━━━━━━━━━━━");
    }

    // 목표 포즈로부터 yaw 각도 계산
    double computeYaw(const geometry_msgs::msg::Pose& pose)
    {
        return std::atan2(pose.position.y, pose.position.x);
    }

    void playTrajectory(const trajectory_msgs::msg::JointTrajectory& traj)
    {
        for(size_t i = 0; i < traj.points.size(); i++)
        {
            auto& p = traj.points[i];
            publish_joint_command(p.positions);

            if(i + 1 < traj.points.size())
            {
                double t1 = rclcpp::Duration(p.time_from_start).seconds();
                double t2 = rclcpp::Duration(traj.points[i+1].time_from_start).seconds();
                double dt = t2 - t1;

                if(dt < 0.001) dt = 0.001;
                std::this_thread::sleep_for(std::chrono::duration<double>(dt));
            }
        }
        
        std::this_thread::sleep_for(200ms);
        updateRobotStateFromJointStates();
    }

    // 베이스를 목표 yaw로 회전
    std::vector<double> rotate_base_to_yaw(double yaw_target)
    {
        RCLCPP_INFO(this->get_logger(), "→ 베이스 회전: %.3f rad (%.1f도)", 
                   yaw_target, yaw_target * 180.0 / M_PI);
        
        updateRobotStateFromJointStates();
        
        auto current = move_group_->getCurrentJointValues();
        if (current.empty() || current.size() < 7)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to get current joint values");
            return {};
        }
        
        RCLCPP_INFO(this->get_logger(), "  현재 베이스: %.3f → 목표: %.3f", current[0], yaw_target);
        
        current[0] = yaw_target;

        move_group_->setStartStateToCurrentState();
        move_group_->setJointValueTarget(current);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        auto result = move_group_->plan(plan);
        
        if(result == moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "베이스 회전 planning 성공");
            playTrajectory(plan.trajectory_.joint_trajectory);
            
            std::this_thread::sleep_for(500ms);
            
            std::lock_guard<std::mutex> lock(joint_mutex_);
            if (!current_joint_positions_.empty() && current_joint_positions_.size() == 7)
            {
                double actual_angle = current_joint_positions_[0];
                RCLCPP_INFO(this->get_logger(), 
                           "베이스 이동 완료: %.3f (오차: %.3f)", 
                           actual_angle, std::abs(actual_angle - yaw_target));
                return current_joint_positions_;
            }
            
            return plan.trajectory_.joint_trajectory.points.back().positions;
        }

        RCLCPP_ERROR(this->get_logger(), "✗ 베이스 회전 planning 실패");
        return {};
    }

    // 베이스 조인트 고정 (Constraint 적용)
    void lock_base_joint(double yaw_fixed)
    {
        moveit_msgs::msg::Constraints constraints;
        moveit_msgs::msg::JointConstraint jc;

        jc.joint_name = "panda_joint1";
        jc.position = yaw_fixed;
        jc.tolerance_above = 0.01;  // ±0.3 rad (약 ±17도)
        jc.tolerance_below = 0.01;
        jc.weight = 1.0;

        constraints.joint_constraints.push_back(jc);
        move_group_->setPathConstraints(constraints);
        move_group_->setPlanningTime(20.0);
        
        RCLCPP_INFO(get_logger(), 
                   "베이스 고정: %.3f rad (tolerance: ±%.3f)", 
                   yaw_fixed, 0.3);
    }

    std::vector<double> plan_and_publish(const geometry_msgs::msg::Pose& pose)
    {
        updateRobotStateFromJointStates();
        
        move_group_->setStartStateToCurrentState();
        move_group_->setPoseTarget(pose);

        moveit::planning_interface::MoveGroupInterface::Plan best_plan;
        size_t min_points = SIZE_MAX;
        bool found_valid_plan = false;

        const int num_attempts = 10;

        for(int i=0; i<num_attempts; i++)
        {
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            auto error = move_group_->plan(plan);

            if(error != moveit::core::MoveItErrorCode::SUCCESS)
            {
                if (i == 0) {
                    RCLCPP_WARN(this->get_logger(), "Planning 시도 중...");
                }
                continue;
            }

            size_t n_points = plan.trajectory_.joint_trajectory.points.size();

            if(n_points > 50)
            {
                continue;
            }

            if(n_points < min_points)
            {
                min_points = n_points;
                best_plan = plan;
                found_valid_plan = true;
            }
            
            if (n_points < 20) break;
        }

        if(!found_valid_plan)
        {
            RCLCPP_ERROR(this->get_logger(), "Planning 실패");
            return {};
        }

        auto traj = best_plan.trajectory_.joint_trajectory;
        RCLCPP_INFO(this->get_logger(), "Trajectory 실행 (%zu points)", traj.points.size());
        playTrajectory(traj);

        return traj.points.back().positions;
    }

    // 완전한 시퀀스: 베이스 회전 → 고정 → Planning → Constraint 해제
    std::vector<double> run_sequence(const geometry_msgs::msg::Pose& pose, const std::string& desc)
    {
        // 1. Yaw 각도 계산
        double yaw = computeYaw(pose);

        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "→ %s", desc.c_str());
        RCLCPP_INFO(this->get_logger(), "  목표: (%.3f, %.3f, %.3f), yaw: %.3f rad", 
                   pose.position.x, pose.position.y, pose.position.z, yaw);
        
        // 2. 베이스를 목표 yaw로 회전
        auto rot_result = rotate_base_to_yaw(yaw);
        if (rot_result.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "베이스 회전 실패");
            return {};
        }
        
        // 3. 실제 도달한 베이스 각도로 Constraint 설정
        double actual_base_angle = rot_result[0];
        RCLCPP_INFO(this->get_logger(), 
                   "→ 베이스를 %.3f rad에 고정하고 Cartesian planning 수행", 
                   actual_base_angle);
        
        lock_base_joint(actual_base_angle);
        
        // 4. Cartesian planning 수행
        auto result = plan_and_publish(pose);

        // 5. Constraint 해제
        move_group_->clearPathConstraints();
        RCLCPP_INFO(this->get_logger(), "베이스 고정 해제");
        
        if (!result.empty()) {
            RCLCPP_INFO(this->get_logger(), "✓✓ %s 완료", desc.c_str());
        }
        
        return result;
    }

    void publish_joint_command(const std::vector<double>& arm_joints)
    {
        sensor_msgs::msg::JointState msg;

        msg.name = {
            "panda_joint1","panda_joint2","panda_joint3",
            "panda_joint4","panda_joint5","panda_joint6",
            "panda_joint7",
            "panda_finger_joint1","panda_finger_joint2"
        };

        msg.position = arm_joints;

        if(msg.position.size() < 9)
        {
            msg.position.push_back(current_gripper_);
            msg.position.push_back(current_gripper_);
        }

        msg.velocity.resize(9, 0.0);
        msg.effort.resize(9, 0.0);

        msg.header.stamp = now();
        joint_pub_->publish(msg);
    }

    void close_gripper(const std::vector<double>& last_joints)
    {
        RCLCPP_INFO(this->get_logger(), "그리퍼 닫기");
        current_gripper_ = 0.0;
        publish_fixed_gripper(last_joints);
    }

    void open_gripper(const std::vector<double>& last_joints)
    {
        RCLCPP_INFO(this->get_logger(), "그리퍼 열기");
        current_gripper_ = 0.04;
        publish_fixed_gripper(last_joints);
    }

    void publish_fixed_gripper(const std::vector<double>& last_joints)
    {
        sensor_msgs::msg::JointState msg;

        msg.name = {
            "panda_joint1","panda_joint2","panda_joint3",
            "panda_joint4","panda_joint5","panda_joint6",
            "panda_joint7",
            "panda_finger_joint1","panda_finger_joint2"
        };

        msg.position = last_joints;
        if (msg.position.size() < 9)
        {
            msg.position.push_back(current_gripper_);
            msg.position.push_back(current_gripper_);
        }

        msg.velocity.resize(9, 0.0);
        msg.effort.resize(9, 0.0);

        msg.header.stamp = now();
        joint_pub_->publish(msg);

        std::this_thread::sleep_for(150ms);

        msg.header.stamp = now();
        joint_pub_->publish(msg);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MoveItToJointCommand>();

    std::thread([node]() {
        std::this_thread::sleep_for(3s);
        node->initialize();
    }).detach();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}