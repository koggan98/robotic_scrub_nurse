#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/string.hpp>
#include <Eigen/Dense>
#include <thread> 
#include <std_msgs/msg/bool.hpp>


// mit ros2 topic pub /tool_selection std_msgs/msg/String "data: '0'" resetten, 1-5 sind werkzeuge

// gripper_mover true = gripper open
// gripper_mover false = gripper close
// gripper_zeroer true = sensing aktiv
// gripper_zeroer false = sensing inaktiv

class HandPositionFollower : public rclcpp::Node {
public:
    HandPositionFollower()
    : Node("moveit_mover") {
        // OMPL Parameter deklarieren
        this->declare_parameter("robot_description_planning.default_planner_config", "RRTConnect");
        this->declare_parameter("robot_description_planning.default_planner_config.settings", "RRTConnectkConfigDefault");
        // Optional: Timeout und Planungszeit konfigurieren
        this->declare_parameter("robot_description_planning.ompl_planning.timeout", 10.0);
        this->declare_parameter("robot_description_planning.ompl_planning.max_planning_attempts", 10);
        // Kinematik-Plugin deklarieren
        this->declare_parameter("robot_description_kinematics.ur_manipulator.kinematics_solver", "kdl_kinematics_plugin/KDLKinematicsPlugin");
        this->declare_parameter("robot_description_kinematics.ur_manipulator.kinematics_solver_search_resolution", 0.005);
        this->declare_parameter("robot_description_kinematics.ur_manipulator.kinematics_solver_timeout", 0.05);

        // Abonnieren des /hand_position Topics
        hand_position_sub_ = this->create_subscription<geometry_msgs::msg::Pose>("/hand_pose", 10,
        std::bind(&HandPositionFollower::handPositionCallback, this, std::placeholders::_1));
        // Gripper-Status abbonnieren
        gripper_done_sub_ = this->create_subscription<std_msgs::msg::Bool>("/gripper_done", 10,
        std::bind(&HandPositionFollower::gripperDoneCallback, this, std::placeholders::_1));
        // tool_selection abbonieren
        tool_selection_sub_ = this->create_subscription<std_msgs::msg::String>("/tool_selection", 10, 
        std::bind(&HandPositionFollower::toolSelectionCallback, this, std::placeholders::_1));
        // joint_state_buffered abbonieren
        joint_state_buffered_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_state_buffered", 10,
        std::bind(&HandPositionFollower::jointStateBufferedCallback, this, std::placeholders::_1));

        // Gripper mover publisher initialisieren
        gripper_mover_ = this->create_publisher<std_msgs::msg::Bool>("/gripper_mover", 10);
        // Gripper zeroer publisher initialisieren
        gripper_zeroer_ = this->create_publisher<std_msgs::msg::Bool>("/gripper_zeroer", 10);
        // request_joint_state publisher initialisieren
        request_joint_state = this->create_publisher<std_msgs::msg::Bool>("/request_joint_state", 10);

        RCLCPP_INFO(this->get_logger(), "Moveit Mover Node initialized.");
    }

    // Initialisiere die MoveGroupInterface
    void initializeMoveGroupInterface() {
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "ur_manipulator");
        move_group_->setEndEffectorLink("gripper_tip_link");
    
        // Konfiguriere MoveIt Parameter
        RCLCPP_INFO(this->get_logger(), "MoveGroupInterface initialized.");
        
        moveToHomePositionUsingJoints();
        //Gripper öffnen (false)
        RCLCPP_INFO(this->get_logger(), "Opening gripper...");
        publishGripperMover(true);
    }

private:
    geometry_msgs::msg::Point tool_position_;
    geometry_msgs::msg::Quaternion tool_orientation_;
    geometry_msgs::msg::Quaternion handover_orientation_;
    geometry_msgs::msg::Pose hand_pose_;
    geometry_msgs::msg::Pose hand_pose_with_offset;
    geometry_msgs::msg::Vector3 hand_offset_;
    sensor_msgs::msg::JointState latest_joint_state_;
    bool hand_pose_received_ = false;
    bool waiting_for_hand_pose_ = false;
    bool waiting_for_gripper_done_ = false;
    bool tool_has_been_picked_up_ = false;
    bool received_joint_state_ = false; 

    void handPositionCallback(const geometry_msgs::msg::Pose::SharedPtr msg) {
        if (!waiting_for_hand_pose_ || !tool_has_been_picked_up_) {
            RCLCPP_INFO(this->get_logger(), "Ignoring hand pose – not waiting.");
            return;
        }    

        hand_pose_ = *msg;  
        hand_pose_with_offset = *msg;
        hand_pose_with_offset.position.x += hand_offset_.x;
        hand_pose_with_offset.position.y += hand_offset_.y;
        hand_pose_with_offset.position.z += hand_offset_.z;
        RCLCPP_INFO(this->get_logger(), "Hand pose received, proceeding with handover...");
        performHandoverToHandPose();
    }
    
    void toolSelectionCallback(const std_msgs::msg::String::SharedPtr msg) {
        std::string cmd = msg->data;

        if (cmd == "0") {
            RCLCPP_WARN(this->get_logger(), "RESETTING SYSTEM STATE MANUALLY.");
            // Flags zurücksetzen
            hand_pose_received_ = false;
            waiting_for_hand_pose_ = false;
            waiting_for_gripper_done_ = false;
            tool_has_been_picked_up_ = false;
            // Gripper öffnen
            publishGripperMover(true);
            // Gripper-Zeroer deaktivieren
            publishGripperZeroer(false);
            // Zu Home fahren
            moveToHomePositionUsingJoints();
            RCLCPP_INFO(this->get_logger(), "System reset complete.");
            return;
    
        } else if (waiting_for_hand_pose_) {
            RCLCPP_WARN(this->get_logger(), "Already waiting for hand pose – ignoring tool command.");
            return;
        } else if (cmd == "1") { // Pinzette
            tool_position_.x = 0.07;
            tool_position_.y = -0.381;
            tool_position_.z = 0.025;
            tool_orientation_.x = -0.7071; // geht auch umgekehrt mit +-
            tool_orientation_.y = 0.7071;
            tool_orientation_.z = 0;
            tool_orientation_.w = 0;
            hand_offset_.x = 0.02;
            hand_offset_.y = 0.0;
            hand_offset_.z = 0.05;
            handover_orientation_.x = -0.63;
            handover_orientation_.y = 0.63;
            handover_orientation_.z = -0.321;
            handover_orientation_.w = 0.321;
            grabTool(tool_position_.x, tool_position_.y, tool_position_.z);
        } else if (cmd == "2") { // Hammer 
            tool_position_.x = 0.131;
            tool_position_.y = -0.345;
            tool_position_.z = 0.00;
            tool_orientation_.x = 0;
            tool_orientation_.y = 1;
            tool_orientation_.z = 0;
            tool_orientation_.w = 0;
            hand_offset_.x = -0.08;
            hand_offset_.y = 0.0;
            hand_offset_.z = 0.05;
            handover_orientation_.x = -0.63; //0.5;
            handover_orientation_.y = 0.63; //-0.5; // für umgekehert ohne -
            handover_orientation_.z = -0.321; //0.5;
            handover_orientation_.w = 0.321; //-0.5; // für umgekehert ohne -
            grabHammer(tool_position_.x, tool_position_.y, tool_position_.z);
        } else if (cmd == "3") { // Schere lang
            tool_position_.x = 0.035;
            tool_position_.y = -0.47;
            tool_position_.z = 0.03;
            tool_orientation_.x = 0;
            tool_orientation_.y = 1;
            tool_orientation_.z = 0;
            tool_orientation_.w = 0;
            hand_offset_.x = -0.05;
            hand_offset_.y = 0.0;
            hand_offset_.z = 0.02;
            handover_orientation_.x = 0.5;
            handover_orientation_.y = -0.5; // für umgekehert ohne -
            handover_orientation_.z = 0.5;
            handover_orientation_.w = -0.5; // für umgekehert ohne -
            grabLongScissor(tool_position_.x, tool_position_.y, tool_position_.z);
        } else if (cmd == "4") { // Schere kurz linke Hand
            tool_position_.x = -0.03;
            tool_position_.y = -0.33;
            tool_position_.z = 0.03;
            tool_orientation_.x = -0.7071; // geht auch umgekehrt mit +-
            tool_orientation_.y = 0.7071;
            tool_orientation_.z = 0;
            tool_orientation_.w = 0;
            hand_offset_.x = 0.0;
            hand_offset_.y = 0.05;
            hand_offset_.z = 0.05;
            handover_orientation_.x = 0.0;
            handover_orientation_.y = 0.891;
            handover_orientation_.z = 0.0;
            handover_orientation_.w = 0.454;
            grabTool(tool_position_.x, tool_position_.y, tool_position_.z);
        } else if (cmd == "5") { // Schere kurz rechte Hand
            tool_position_.x = -0.03;
            tool_position_.y = -0.33;
            tool_position_.z = 0.03;
            tool_orientation_.x = 0.7071; // geht auch umgekehrt mit +-
            tool_orientation_.y = 0.7071;
            tool_orientation_.z = 0;
            tool_orientation_.w = 0;
            hand_offset_.x = 0.0;
            hand_offset_.y = -0.05;
            hand_offset_.z = 0.05;
            handover_orientation_.x = 0.0;
            handover_orientation_.y = 0.891;
            handover_orientation_.z = 0.0;
            handover_orientation_.w = 0.454;
            grabTool(tool_position_.x, tool_position_.y, tool_position_.z);
        } else if (cmd == "6") { // Haken klein
            tool_position_.x = 0.1265;
            tool_position_.y = -0.39;
            tool_position_.z = 0.015;
            tool_orientation_.x = 1;
            tool_orientation_.y = 0;
            tool_orientation_.z = 0;
            tool_orientation_.w = 0;
            hand_offset_.x = -0.07;
            hand_offset_.y = 0.0;
            hand_offset_.z = 0.07;
            handover_orientation_.x = -0.63;
            handover_orientation_.y = 0.63;
            handover_orientation_.z = -0.321;
            handover_orientation_.w = 0.321;
            grabRetractor(tool_position_.x, tool_position_.y, tool_position_.z);
        } else {
            RCLCPP_WARN(this->get_logger(), "Unknown command: '%s'", cmd.c_str());
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Waiting for hand pose...");
    }

    void performHandoverToHandPose() {
        // Fahre über die Hand
        geometry_msgs::msg::Pose target_pose = hand_pose_with_offset;
        target_pose.orientation = handover_orientation_;
        move_group_->setPlanningTime(1.0);
        move_group_->setMaxVelocityScalingFactor(0.6);
        move_group_->setMaxAccelerationScalingFactor(0.6);
        move_group_->setPoseTarget(target_pose);
        RCLCPP_INFO(this->get_logger(), "Target for handover: x=%.2f y=%.2f z=%.2f", 
            target_pose.position.x, 
            target_pose.position.y, 
            target_pose.position.z);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Moving above hand...");
            move_group_->execute(plan);
            waiting_for_hand_pose_ = false;
            // Öffne den Greifer über /gripper_zeroer
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            RCLCPP_INFO(this->get_logger(), "Activating gripper sensing...");
            publishGripperZeroer(true);
            waiting_for_gripper_done_ = true;
            hand_pose_received_ = true;
            std::this_thread::sleep_for(std::chrono::milliseconds(1500));
        } else {
            RCLCPP_ERROR(this->get_logger(), "Planning to above hand failed.");
            return;
        }
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr tool_selection_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gripper_done_sub_; 
    void gripperDoneCallback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
            RCLCPP_INFO(this->get_logger(), "Gripper opened, returning to home.");
            moveToHomePositionUsingJoints();
            // Reset
            hand_pose_received_ = false;
            waiting_for_hand_pose_ = false;
            tool_has_been_picked_up_ = false;
            RCLCPP_INFO(this->get_logger(), "Ready for next tool command.");
        }
    }   

    void jointStateBufferedCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    latest_joint_state_ = *msg;
    received_joint_state_ = true;
    RCLCPP_INFO(this->get_logger(), "Received joint_state from Python relay.");
    }
    
    // Bewegung zur Home-Position über Joint-Winkel
    void moveToHomePositionUsingJoints() {
        // Definiere Joint-Winkel für die Home-Position
        std::vector<double> home_joint_positions = {
            0,    // Joint 1: 0°
            -2.486, // Joint 2: 90°
            1.227,    // Joint 3: 0°
            -1.294,    // Joint 4: 0°
            -M_PI_2,    // Joint 5: 0°
            0.0     // Joint 6: 0°
        };
        // Setze Joint-Winkel als Ziel
        move_group_->setPlanningTime(1.0);
        move_group_->setMaxVelocityScalingFactor(1);
        move_group_->setMaxAccelerationScalingFactor(1);
        move_group_->setJointValueTarget(home_joint_positions);

        // Bewegung planen und ausführen
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Planning to Home-Position (Joints) successful, executing...");
            move_group_->execute(plan);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Planning to Home-Position (Joints) failed.");
        }
    }

        void grabTool(double x, double y, double z) {
        RCLCPP_INFO(this->get_logger(), "Moving to object at x=%.2f y=%.2f z=%.2f", x, y, z);
        // Fahre über Werkzeug
        publishGripperMover(true);
        geometry_msgs::msg::Pose object_pose;
        object_pose.position.x = x;
        object_pose.position.y = y;
        object_pose.position.z = z;
        object_pose.orientation = tool_orientation_;
        geometry_msgs::msg::Pose lift_pose = object_pose;
        lift_pose.position.z += 0.1;
        move_group_->setPlanningTime(1.0);
        move_group_->setMaxVelocityScalingFactor(1);
        move_group_->setMaxAccelerationScalingFactor(1);
        move_group_->setPoseTarget(lift_pose);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Planning above object successful, executing...");
            move_group_->execute(plan);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Planning above object failed.");
            return;
        }
        // Senke auf Werkzeug ab
        move_group_->setPlanningTime(1.0);
        move_group_->setMaxVelocityScalingFactor(0.5);
        move_group_->setMaxAccelerationScalingFactor(0.5);
        move_group_->setPoseTarget(object_pose);
        if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Planning to object successful, executing...");
            move_group_->execute(plan);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Planning to object failed.");
            return;
        }
        // Greifer schließen
        RCLCPP_INFO(this->get_logger(), "Closing gripper on object...");
        publishGripperMover(false);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        tool_has_been_picked_up_ = true;
        // Über Werkzeug fahren
        move_group_->setPlanningTime(1.0);
        move_group_->setMaxVelocityScalingFactor(1);
        move_group_->setMaxAccelerationScalingFactor(1);
        move_group_->setPoseTarget(lift_pose);
        if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Lifting object...");
            move_group_->execute(plan);
            waiting_for_hand_pose_ = true;
            moveToHomePositionUsingJoints();
        } else {
            RCLCPP_ERROR(this->get_logger(), "Lift motion failed.");
        }
    }

        void grabHammer(double x, double y, double z) {
        RCLCPP_INFO(this->get_logger(), "Moving to object at x=%.2f y=%.2f z=%.2f", x, y, z);
        // Fahre über Werkzeug
        publishGripperMover(true);
        geometry_msgs::msg::Pose object_pose;
        object_pose.position.x = x;
        object_pose.position.y = y;
        object_pose.position.z = z;
        object_pose.orientation = tool_orientation_;
        geometry_msgs::msg::Pose lift_pose = object_pose;
        lift_pose.position.z += 0.1;
        move_group_->setPlanningTime(1.0);
        move_group_->setMaxVelocityScalingFactor(1);
        move_group_->setMaxAccelerationScalingFactor(1);
        move_group_->setPoseTarget(lift_pose);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Planning above object successful, executing...");
            move_group_->execute(plan);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Planning above object failed.");
            return;
        }
        // Senke auf Werkzeug ab
        move_group_->setPlanningTime(1.0);
        move_group_->setMaxVelocityScalingFactor(1);
        move_group_->setMaxAccelerationScalingFactor(1);
        move_group_->setPoseTarget(object_pose);
        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(object_pose);  // Zielpose z. B. abgesenkt
        moveit_msgs::msg::RobotTrajectory trajectory;
        const double eef_step = 0.005;  // Schrittweite in m (z. B. 5mm)
        const double jump_threshold = 0.0;  // keine plötzlichen Sprünge erlauben

        double fraction = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        if (fraction > 0.99) {
            RCLCPP_INFO(this->get_logger(), "Linear path to object (%.2f%% achieved), executing...", fraction * 100.0);
            moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
            cartesian_plan.trajectory_ = trajectory;
            move_group_->execute(cartesian_plan);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to compute linear Cartesian path (only %.2f%% achieved)", fraction * 100.0);
            return;
        }
        // Greifer schließen
        RCLCPP_INFO(this->get_logger(), "Closing gripper on object...");
        publishGripperMover(false);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        tool_has_been_picked_up_ = true;
        // Über Werkzeug fahren (linear mit computeCartesianPath)
        std::vector<geometry_msgs::msg::Pose> lift_waypoints;
        lift_waypoints.push_back(lift_pose);  // Zielpose über dem Objekt

        moveit_msgs::msg::RobotTrajectory lift_trajectory;
        const double eef_step_lift = 0.005;  // Schrittweite z. B. 5mm
        const double jump_threshold_lift = 0.0;  // keine Sprünge erlaubt

        double lift_fraction = move_group_->computeCartesianPath(
            lift_waypoints, eef_step_lift, jump_threshold_lift, lift_trajectory);

        if (lift_fraction > 0.99) {
            RCLCPP_INFO(this->get_logger(), "Lifting object (linear path %.2f%% achieved)...", lift_fraction * 100.0);
            moveit::planning_interface::MoveGroupInterface::Plan lift_plan;
            lift_plan.trajectory_ = lift_trajectory;
            move_group_->execute(lift_plan);

            waiting_for_hand_pose_ = true;
            moveToHomePositionUsingJoints();
        } else {
            RCLCPP_ERROR(this->get_logger(), "Cartesian lift motion failed (%.2f%% achieved)", lift_fraction * 100.0);
        }
    }

    void grabLongScissor(double x, double y, double z) {
        RCLCPP_INFO(this->get_logger(), "Moving to object at x=%.2f y=%.2f z=%.2f", x, y, z);
        // Fahre über Werkzeug
        publishGripperMover(true);
        geometry_msgs::msg::Pose object_pose;
        object_pose.position.x = x;
        object_pose.position.y = y;
        object_pose.position.z = z;
        object_pose.orientation = tool_orientation_;
        std::vector<double> over_scissor_position = {
        -1.80589,
        -1.46608,
        1.51407,
        -1.98042,
        -1.534842,
        -0.205774
        };
        // Setze Joint-Winkel als Ziel
        move_group_->setPlanningTime(1.0);
        move_group_->setMaxVelocityScalingFactor(1);
        move_group_->setMaxAccelerationScalingFactor(1);
        move_group_->setJointValueTarget(over_scissor_position);

        // Bewegung planen und ausführen
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Planning above Scissor successful, executing...");
            move_group_->execute(plan);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Planning above Scissor failed.");
        }
        // Senke auf Werkzeug ab
        // constraints definieren
        moveit_msgs::msg::Constraints constraints;
        // Elbow up constraints hinzufügen (elbow joint muss positiv sein)
        moveit_msgs::msg::JointConstraint elbow_constraint;
        elbow_constraint.joint_name = "elbow_joint"; //
        elbow_constraint.position = 0.8042477;         // Zielposition (ca. 46.08°)
        elbow_constraint.tolerance_above = 0.7853;       // darf bis 45° hoch gehen
        elbow_constraint.tolerance_below = 0.7853;       // darf bis 45° runter gehen
        elbow_constraint.weight = 1.0;
        constraints.joint_constraints.push_back(elbow_constraint);
        move_group_->setPathConstraints(constraints);
        move_group_->setPlanningTime(1.0);
        move_group_->setMaxVelocityScalingFactor(1);
        move_group_->setMaxAccelerationScalingFactor(1);
        move_group_->setPoseTarget(object_pose);
        if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Planned motion to object pose successful, executing...");
            move_group_->execute(plan);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Planning to object pose failed.");
            return;
        }
        move_group_->clearPathConstraints();
        // Greifer schließen
        RCLCPP_INFO(this->get_logger(), "Closing gripper on object...");
        publishGripperMover(false);
        tool_has_been_picked_up_ = true;
        // Vor Werkzeug fahren
        std::vector<double> after_scissor_position = {
        -1.79943,
        -1.33028,
        1.62385,
        -2.12720,
        -1.55281,
        -0.205774
        };
        move_group_->setPlanningTime(1.0);
        move_group_->setMaxVelocityScalingFactor(1);
        move_group_->setMaxAccelerationScalingFactor(1);
        move_group_->setJointValueTarget(after_scissor_position);

        // Bewegung planen und ausführen
        if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Planning above Scissor successful, executing...");
            move_group_->execute(plan);
            waiting_for_hand_pose_ = true;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Planning above Scissor failed.");
        }
        // über Werkzeug fahren
        move_group_->setPlanningTime(1.0);
        move_group_->setMaxVelocityScalingFactor(1);
        move_group_->setMaxAccelerationScalingFactor(1);
        move_group_->setJointValueTarget(over_scissor_position);

        // Bewegung planen und ausführen
        if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Planning above Scissor successful, executing...");
            move_group_->execute(plan);
            waiting_for_hand_pose_ = true;
            moveToHomePositionUsingJoints();
        } else {
            RCLCPP_ERROR(this->get_logger(), "Planning above Scissor failed.");
        }
    }

    void grabRetractor(double x, double y, double z) {
        RCLCPP_INFO(this->get_logger(), "Moving to object at x=%.2f y=%.2f z=%.2f", x, y, z);
        // Fahre über Werkzeug
        publishGripperMover(true);
        geometry_msgs::msg::Pose object_pose;
        object_pose.position.x = x;
        object_pose.position.y = y;
        object_pose.position.z = z;
        object_pose.orientation = tool_orientation_;
        geometry_msgs::msg::Pose lift_pose = object_pose;
        lift_pose.position.z += 0.15;
        move_group_->setPlanningTime(1.0);
        move_group_->setMaxVelocityScalingFactor(1);
        move_group_->setMaxAccelerationScalingFactor(1);
        move_group_->setPoseTarget(lift_pose);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Planning above object successful, executing...");
            move_group_->execute(plan);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Planning above object failed.");
            return;
        }
        // Senke auf Werkzeug ab
        move_group_->setPlanningTime(1.0);
        move_group_->setMaxVelocityScalingFactor(1);
        move_group_->setMaxAccelerationScalingFactor(1);
        move_group_->setPoseTarget(object_pose);
        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(object_pose);  // Zielpose z. B. abgesenkt
        moveit_msgs::msg::RobotTrajectory trajectory;
        const double eef_step = 0.005;  // Schrittweite in m (z. B. 5mm)
        const double jump_threshold = 0.0;  // keine plötzlichen Sprünge erlauben

        double fraction = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        if (fraction > 0.99) {
            RCLCPP_INFO(this->get_logger(), "Linear path to object (%.2f%% achieved), executing...", fraction * 100.0);
            moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
            cartesian_plan.trajectory_ = trajectory;
            move_group_->execute(cartesian_plan);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to compute linear Cartesian path (only %.2f%% achieved)", fraction * 100.0);
            return;
        }
        // Greifer schließen
        RCLCPP_INFO(this->get_logger(), "Closing gripper on object...");
        publishGripperMover(false);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        tool_has_been_picked_up_ = true;
        // Über Werkzeug fahren (linear mit computeCartesianPath)
        std::vector<geometry_msgs::msg::Pose> lift_waypoints;
        lift_waypoints.push_back(lift_pose);  // Zielpose über dem Objekt

        moveit_msgs::msg::RobotTrajectory lift_trajectory;
        const double eef_step_lift = 0.005;  // Schrittweite z. B. 5mm
        const double jump_threshold_lift = 0.0;  // keine Sprünge erlaubt

        double lift_fraction = move_group_->computeCartesianPath(
            lift_waypoints, eef_step_lift, jump_threshold_lift, lift_trajectory);

        if (lift_fraction > 0.99) {
            RCLCPP_INFO(this->get_logger(), "Lifting object (linear path %.2f%% achieved)...", lift_fraction * 100.0);
            moveit::planning_interface::MoveGroupInterface::Plan lift_plan;
            lift_plan.trajectory_ = lift_trajectory;
            move_group_->execute(lift_plan);

            waiting_for_hand_pose_ = true;
            moveToHomePositionUsingJoints();
        } else {
            RCLCPP_ERROR(this->get_logger(), "Cartesian lift motion failed (%.2f%% achieved)", lift_fraction * 100.0);
        }
    }

    void moveToModifiedJoints() {
        received_joint_state_ = false;
        publishRequestJointState(true);
        RCLCPP_INFO(this->get_logger(), "Published joint state request.");
        // Starte eigenen Thread für Wartezeit, um Callback nicht zu blockieren
        std::thread([this]() {
            int retries = 0;
            while (!received_joint_state_ && rclcpp::ok() && retries < 30) {
                rclcpp::sleep_for(std::chrono::milliseconds(100));
                RCLCPP_WARN(this->get_logger(), "Waiting for joint state... (%d)", retries);
                retries++;
            }
            if (!received_joint_state_) {
                RCLCPP_ERROR(this->get_logger(), "Still no joint state received.");
                return;
            }
            this->planAndExecuteFromBufferedState();
        }).detach();
    }

    void planAndExecuteFromBufferedState() {
        // Map von Joint-Name zu Wert erstellen
        std::map<std::string, double> joint_map;
        for (size_t i = 0; i < latest_joint_state_.name.size(); ++i) {
            joint_map[latest_joint_state_.name[i]] = latest_joint_state_.position[i];
        }

        // Gelenkwinkel gezielt setzen
        joint_map["wrist_2_joint"] = 0.0;
        joint_map["wrist_3_joint"] = 0.0;

        // Gewünschte Reihenfolge gemäß URDF
        std::vector<std::string> ur_joint_order = {
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"
        };

        // Zielwinkel aufbauen und loggen
        std::vector<double> joint_positions;
        for (const std::string& name : ur_joint_order) {
            double rad = joint_map[name];
            double deg = rad * 180.0 / M_PI;
            RCLCPP_INFO(this->get_logger(), "Target Joint %s: %.4f rad (%.2f°)", name.c_str(), rad, deg);
            joint_positions.push_back(rad);
        }

        // Planen & Ausführen
        move_group_->setPlanningTime(0.5);
        move_group_->setMaxVelocityScalingFactor(1.0);
        move_group_->setMaxAccelerationScalingFactor(1.0);
        move_group_->setJointValueTarget(joint_positions);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Modified joint movement planned successfully, executing...");
            move_group_->execute(plan);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Planning failed.");
        }
    }

    void publishGripperMover(bool close) {
        auto msg = std_msgs::msg::Bool();
        msg.data = close;
        gripper_mover_->publish(msg);
    }

    void publishGripperZeroer(bool close) {
        auto msg = std_msgs::msg::Bool();
        msg.data = close;
        gripper_zeroer_->publish(msg);
    }

    void publishRequestJointState(bool trigger) {
        auto msg = std_msgs::msg::Bool();
        msg.data = trigger;
        request_joint_state->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published joint state request.");
    }

    // Subscriber für /hand_position Topic
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr hand_position_sub_;
    // MoveGroupInterface für Robotersteuerung
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    // Publisher für Gripper-Befehle
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gripper_mover_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gripper_zeroer_;
    // Subscriber für /joint_state_buffered
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_buffered_sub_;
    // Publisher für /request_joint_states
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr request_joint_state;
};

int main(int argc, char** argv) {
    // ROS 2 initialisieren
    rclcpp::init(argc, argv);
    // Node erstellen und MoveGroupInterface initialisieren
    auto node = std::make_shared<HandPositionFollower>();
    node->initializeMoveGroupInterface();
    // Node ausführen
    rclcpp::spin(node);
    // ROS 2 beenden
    rclcpp::shutdown();
    return 0;
}
