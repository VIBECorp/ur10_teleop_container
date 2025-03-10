// Includes
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tutorial_interfaces/srv/set_target_pose.hpp"
#include "tf2_eigen/tf2_eigen.hpp" // Eigen::fromMsg
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include <moveit/move_group_interface/move_group_interface.hpp> // MoveGroupInterface

// https://github.com/moveit/moveit2_tutorials/blob/main/doc/examples/robot_model_and_robot_state/src/robot_model_and_robot_state_tutorial.cpp 
#include "moveit/robot_model_loader/robot_model_loader.hpp"
#include "moveit/robot_model/robot_model.hpp"
#include "moveit/robot_state/robot_state.hpp"


// Mode
const int INIT = 0;
const int TELEOP = 1;
const int TASK_CONTROL = 2;
const int JOINT_CONTROL = 3;
const int AI = 4;
const int MOVEIT = 5;
const int IDLE = 6;

// Node class
class Task2JointNode : public rclcpp::Node {
public:
    Task2JointNode(
        robot_model_loader::RobotModelLoader robot_model_loader,
        const moveit::core::RobotModelPtr& kinematic_model) 
    : Node("task2joint_node"), robot_model_loader(robot_model_loader), kinematic_model(kinematic_model)
    {
        RCLCPP_INFO(this->get_logger(), "Task2Joint Node started");

        // Get kinematic model, robot state, joint model group interface
        //PLANNING_GROUP = "ur10_arm";
        //moveit::planning_interface::MoveGroupInterface move_group(this, PLANNING_GROUP);
        RCLCPP_INFO(this->get_logger(), "Model frame: %s", kinematic_model->getModelFrame().c_str());
        robot_state = std::make_shared<moveit::core::RobotState>(kinematic_model);
        joint_model_group = kinematic_model->getJointModelGroup(PLANNING_GROUP);
        //const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
        joint_names = joint_model_group->getVariableNames();

        // Get Joint Values
        // robot_state->copyJointGroupPositions(joint_model_group, joint_values);
        // for (std::size_t i = 0; i < joint_names.size(); ++i)
        // {
        //     RCLCPP_INFO(this->get_logger(), "Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
        // }

        // Initialize Publishers & Subscribers
        ik_result_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("ik_result", 10);
        ik_success_pub_ = this->create_publisher<std_msgs::msg::Bool>("ik_success", 10);
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10, std::bind(&Task2JointNode::joint_state_callback, this, std::placeholders::_1)
        );
        target_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "target_pose", 10, std::bind(&Task2JointNode::target_pose_callback, this, std::placeholders::_1)
        );
        mode_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "mode", 10, std::bind(&Task2JointNode::mode_callback, this, std::placeholders::_1)
        );

        // Initialize Services & Clients
        set_target_pose_client_ = this->create_client<tutorial_interfaces::srv::SetTargetPose>("set_target_pose");

        // Initialize Variables
        current_joint_position.assign(6, 0.0);
        current_joint_velocity.assign(6, 0.0);
        ik_result = {0.0, -1.8397, 2.03, -0.2435, 1.5615, 0.0};
        pre_ik_result = {0.0, -1.8397, 2.03, -0.2435, 1.5615, 0.0};

        // Listen Current Pose TF
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Initialize loop
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(4),
            std::bind(&Task2JointNode::loop, this)
        );
    }

private:
    // Publishers & Subscribers
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr ik_result_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ik_success_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr mode_sub_;

    // Services & Clients
    rclcpp::Client<tutorial_interfaces::srv::SetTargetPose>::SharedPtr set_target_pose_client_;
    
    // Timer for loop
    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Variables
    const std::string PLANNING_GROUP = "ur_manipulator";
    robot_model_loader::RobotModelLoader robot_model_loader;
    const moveit::core::RobotModelPtr& kinematic_model;
    moveit::core::RobotStatePtr robot_state;
    const moveit::core::JointModelGroup* joint_model_group;
    std::vector<std::string> joint_names;
    std::vector<double> current_joint_position, current_joint_velocity, joint_values, ik_result, pre_ik_result;
    sensor_msgs::msg::JointState current_joint_state;
    geometry_msgs::msg::PoseStamped current_target_pose, last_success_target_pose;
    std_msgs::msg::Float64MultiArray current_pose_xyzrpy;
    int mode = INIT;
    double continuity_threshold = 0.3;

    // Callbacks
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        this->current_joint_state = *msg;
        this->current_joint_position = current_joint_state.position;
        this->current_joint_velocity = current_joint_state.velocity;
        // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", this->current_joint_state.name[0].c_str());
        // RCLCPP_INFO(this->get_logger(), "I heard: '%f' '%f", this->current_joint_position[0], this->current_joint_velocity[0]);
    }

    void target_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        this->current_target_pose = *msg;
        // RCLCPP_INFO(this->get_logger(), "I heard: '%f'", this->current_target_pose.pose.position.x);
    }

    void mode_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        this->mode = msg->data;
    }

    // Methods
    bool solve_ik(geometry_msgs::msg::Pose target_pose){
        // set joint position to current state
        robot_state->setJointGroupPositions(joint_model_group, current_joint_position);
        // enforce joint limits
        Eigen::Isometry3d pose_in;
        robot_state->enforceBounds();
        Eigen::fromMsg(target_pose, pose_in); // https://docs.ros2.org/foxy/api/tf2_eigen/namespaceEigen.html
        
        bool found_ik = robot_state->setFromIK(joint_model_group, pose_in, 0.1);
        robot_state->update(); // https://github.com/ros-planning/moveit/pull/188
        
        // Now, we can print out the IK solution (if found):
        if (found_ik)
        {
            robot_state->copyJointGroupPositions(joint_model_group, ik_result);
            // RCLCPP_INFO(this->get_logger(), "found IK solution");
            
            // Check solution continuity
            bool continuity = check_solution_continuity();
            if(!continuity){
                found_ik = false;
                //RCLCPP_INFO(this->get_logger(), "IK solution is not continuous");
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Did not find IK solution");
        }
        return found_ik;
    }

    bool check_solution_continuity(void){ // check if the solution is continuous with the previous solution
        bool solution_continuity = true;
        for(int i=0; i<int(joint_names.size()); i++){
          if (abs(ik_result[i]-pre_ik_result[i]) > continuity_threshold)
            solution_continuity = false;
        }
        return solution_continuity;
    }

    geometry_msgs::msg::TransformStamped listenTransform() {
        geometry_msgs::msg::TransformStamped transform;
        try {
            transform = tf_buffer_->lookupTransform("base_link", "wrist_3_link", tf2::TimePointZero);
        } catch (const tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "Could not transform: %s", ex.what());
        }
        return transform;
    }

    void updateCurrentPoseRPY(void)
    {
        geometry_msgs::msg::TransformStamped transform = listenTransform();

        // 1️⃣ 위치 (x, y, z) 추출
        double x = transform.transform.translation.x;
        double y = transform.transform.translation.y;
        double z = transform.transform.translation.z;

        // 2️⃣ 회전 (roll, pitch, yaw) 변환 (쿼터니언 → RPY)
        tf2::Quaternion q(transform.transform.rotation.x,
                          transform.transform.rotation.y,
                          transform.transform.rotation.z,
                          transform.transform.rotation.w);

        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        // 3️⃣ Float64MultiArray 메시지에 저장
        
        current_pose_xyzrpy.data = {x, y, z, roll, pitch, yaw};
        // RCLCPP_INFO(this->get_logger(), "Current Pose: %f %f %f %f %f %f", x, y, z, roll, pitch, yaw);
    }

    void publish_ik_result(std::vector<double> ik_result) {
        std_msgs::msg::Float64MultiArray ik_result_msg;
        for (std::size_t i = 0; i < joint_names.size(); ++i)
        {
            ik_result_msg.data.push_back(ik_result[i]);
        }
        ik_result_pub_->publish(ik_result_msg);
    }

    void loop() {
        // RCLCPP_INFO(this->get_logger(), "Current mode: %d", mode);
        // for (std::size_t i = 0; i < joint_names.size(); ++i)
        // {
        //     RCLCPP_INFO(this->get_logger(), "Joint %ld %s", i, joint_names[i].c_str());
        // }
        updateCurrentPoseRPY();
        if (mode == TELEOP || mode == TASK_CONTROL || mode == AI) {
            // Solve IK
            // RCLCPP_INFO(this->get_logger(), "Solve IK");
            bool success = solve_ik(current_target_pose.pose);

            // If success, publish ik_result message
            if (success) {
                // Publish ik_result message
                publish_ik_result(ik_result);
                // Store result for continuity check in the next iteration
                pre_ik_result = ik_result;
                // last_success_target_pose = current_target_pose;
            } // If not success, keep the last valid target pose
            else {
                // Keep valid target pose
                auto request = std::make_shared<tutorial_interfaces::srv::SetTargetPose::Request>();

                request->target_pose = current_pose_xyzrpy;
                auto result = set_target_pose_client_->async_send_request(request);
                // Publish pre_ik_result message
                publish_ik_result(pre_ik_result);
            }
            // Publish ik_success message
            std_msgs::msg::Bool ik_success_msg;
            ik_success_msg.data = success;
            ik_success_pub_->publish(ik_success_msg);
        } else if (mode == INIT) {
            // RCLCPP_INFO(this->get_logger(), "INIT");
            ik_result = {0.0, -1.8397, 2.03, -0.2435, 1.5615, 0.0}; // must be joint_states of INIT pose
            pre_ik_result = {0.0, -1.8397, 2.03, -0.2435, 1.5615, 0.0}; // must be joint_states of INIT pose
        }

    }
};

// Main function
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    node_options.parameter_overrides({{"use_sim_time", rclcpp::ParameterValue(true)}});
    auto move_group_node = rclcpp::Node::make_shared("move_group_interface_cpp", node_options);
    robot_model_loader::RobotModelLoader robot_model_loader(move_group_node);
    const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();

    auto node = std::make_shared<Task2JointNode>(robot_model_loader, kinematic_model);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}