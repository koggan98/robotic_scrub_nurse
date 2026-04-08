#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <Eigen/Dense>
#include <thread> 
#include <std_msgs/msg/bool.hpp>


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
        RCLCPP_INFO(this->get_logger(), "MoveGroupInterface initialized.");
        
        move_group_->setEndEffectorLink("gripper_tip_link");
        // Gripper öffnen (false)
        RCLCPP_INFO(this->get_logger(), "Opening gripper...");
        publishGripperMover(false);
        //moveToModifiedJoints();


        //std::this_thread::sleep_for(std::chrono::seconds(1));  // Kurze Wartezeit für den Gripper
        //moveToObjectPosition(0.4, 0.2, 0.27);

        

        // Fahre zur Startposition
        //moveToObjectPosition();

        // Gripper schließen (true)
        //RCLCPP_INFO(this->get_logger(), "Closing gripper...");
        //publishGripperCommand(true);
        //std::this_thread::sleep_for(std::chrono::seconds(1));  // Kurze Wartezeit für den Gripper

        // Über das Objekt fahren
        //moveAboveObject();
    }

private:
    void handPositionCallback(const geometry_msgs::msg::Pose::SharedPtr msg) { // Plant bewegung wenn Handposition empfangen wird
        // Zielpose definieren
        geometry_msgs::msg::Pose target_pose;
        double x_pos = msg->position.x;
        double y_pos = msg->position.y;
        double z_pos = msg->position.z;
        RCLCPP_INFO(this->get_logger(), "Received hand position: x=%.3f, y=%.3f, z=%.3f", x_pos, y_pos, z_pos);
        target_pose.position.x = x_pos; 
        target_pose.position.y = y_pos;
        target_pose.position.z = z_pos;
        target_pose.orientation = msg->orientation;
        //target_pose.orientation.x = -0.63;
        //target_pose.orientation.y = 0.63;
        //target_pose.orientation.z = -0.321;
        //target_pose.orientation.w = 0.321;
        
        //target_pose.orientation.x = 1;
        //target_pose.orientation.y = 0;
        //target_pose.orientation.z = 0;
        //target_pose.orientation.w = 0;

        // constraints definieren
        moveit_msgs::msg::Constraints constraints;
        // Elbow up constraints hinzufügen (elbow joint muss positiv sein)
        moveit_msgs::msg::JointConstraint elbow_constraint;
        elbow_constraint.joint_name = "elbow_joint"; //
        elbow_constraint.position = 0.52;              // Zielposition (ca. 30°)
        elbow_constraint.tolerance_above = 2.25;       // darf bis 3 rad also ca. 170° nach oben gehen
        elbow_constraint.tolerance_below = 0.52;       // erlaubt nur positive Winkel > 0
        elbow_constraint.weight = 1.0;
        constraints.joint_constraints.push_back(elbow_constraint);
        // Shoulder/base constraint zwischen -90° und 90°
        moveit_msgs::msg::JointConstraint shoulder_constraint;
        shoulder_constraint.joint_name = "shoulder_pan_joint"; // ggf. anpassen
        shoulder_constraint.position = 0;                  
        shoulder_constraint.tolerance_above = 1.57;            
        shoulder_constraint.tolerance_below = -1.57;           
        shoulder_constraint.weight = 1.0;
        constraints.joint_constraints.push_back(shoulder_constraint);
        // Constraint setzen
        // move_group_->setPathConstraints(constraints);
        // Zielpose setzen
        // Konfiguriere MoveIt Parameter
        move_group_->setPlanningTime(0.5);
        move_group_->setMaxVelocityScalingFactor(0.5);
        move_group_->setMaxAccelerationScalingFactor(0.5);
        move_group_->setPoseTarget(target_pose);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Moving to hand");
            move_group_->execute(plan);
        
            // Greiferaktivierung starten
            RCLCPP_INFO(this->get_logger(), "Zeroing gripper after reaching target pose...");
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
            publishGripperZeroer(true);
            ////////////////////////////////////////////////////////////////////////////////////////////////////
            RCLCPP_INFO(this->get_logger(), "Waiting for feedback via /gripper_done...");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Planning failed.");
        }
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gripper_done_sub_; 
    void gripperDoneCallback(const std_msgs::msg::Bool::SharedPtr msg) { // "Gripper wurde geöffnet, kann jetzt zurück zur Home-Position"
        if (msg->data) {
            RCLCPP_INFO(this->get_logger(), "Force feedback disabled");
            RCLCPP_INFO(this->get_logger(), "Gripper opened – will return to home in 0 second...");
            std::this_thread::sleep_for(std::chrono::seconds(0));
            //moveToHomePositionUsingJoints();
            //publishGripperMover(false);
        }
    }

    // Bewegung zur Home-Position über Joint-Winkel
    void moveToHomePositionUsingJoints() {
        // Definiere Joint-Winkel für die Home-Position
        std::vector<double> home_joint_positions = {
            0,    // Joint 1: 0°
            -M_PI_2, // Joint 2: 90°
            0.0,    // Joint 3: 0°
            0.0,    // Joint 4: 0°
            0.0,    // Joint 5: 0°
            0.0     // Joint 6: 0°
        };

        // Setze Joint-Winkel als Ziel
        move_group_->setPlanningTime(0.5);
        move_group_->setMaxVelocityScalingFactor(1);
        move_group_->setMaxAccelerationScalingFactor(1);
        move_group_->setJointValueTarget(home_joint_positions);

        // Bewegung planen und ausführen
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Planning to Home-Position successful, executing...");
            move_group_->execute(plan);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Planning to Home-Position failed.");
        }
    }

    sensor_msgs::msg::JointState latest_joint_state_;
    bool received_joint_state_ = false;

    void jointStateBufferedCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    latest_joint_state_ = *msg;
    received_joint_state_ = true;
    RCLCPP_INFO(this->get_logger(), "Received joint_state from Python relay.");
    }

    //void moveToModifiedJoints() {
    //    received_joint_state_ = false;
    //    publishRequestJointState(true);
    //    RCLCPP_INFO(this->get_logger(), "Published joint state request.");

        // Starte eigenen Thread für Wartezeit, um Callback nicht zu blockieren
    //    std::thread([this]() {
    //        int retries = 0;
    //        while (!received_joint_state_ && rclcpp::ok() && retries < 30) {
    //            rclcpp::sleep_for(std::chrono::milliseconds(100));
    //            RCLCPP_WARN(this->get_logger(), "Waiting for joint state... (%d)", retries);
    //            retries++;
    //        }
    //        if (!received_joint_state_) {
    //            RCLCPP_ERROR(this->get_logger(), "Still no joint state received.");
    //            return;
    //        }
    //        this->planAndExecuteFromBufferedState();
    //    }).detach();
    //}

    void planAndExecuteFromBufferedState() {
        // Map von Joint-Name zu Wert erstellen
        std::map<std::string, double> joint_map;
        for (size_t i = 0; i < latest_joint_state_.name.size(); ++i) {
            joint_map[latest_joint_state_.name[i]] = latest_joint_state_.position[i];
        }

        // Gelenkwinkel gezielt setzen
        joint_map["wrist_2_joint"] += M_2_PI;

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



    void moveToObjectPosition(double x, double y, double z) {
        RCLCPP_INFO(this->get_logger(), "Moving to object at x=%.2f y=%.2f z=%.2f", x, y, z);
        // 1. Fahre zur Objektposition
        geometry_msgs::msg::Pose object_pose;
        object_pose.position.x = x;
        object_pose.position.y = y;
        object_pose.position.z = z;
        object_pose.orientation.x = 1.0;
        object_pose.orientation.y = 0.0;
        object_pose.orientation.z = 0.0;
        object_pose.orientation.w = 0.0;
    
        move_group_->setPoseTarget(object_pose);
    
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Planning to object successful, executing...");
            move_group_->execute(plan);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Planning to object failed.");
            return;
        }
    
        // 2. Greifer schließen
        RCLCPP_INFO(this->get_logger(), "Closing gripper on object...");
        publishGripperMover(false);
        std::this_thread::sleep_for(std::chrono::seconds(1));  // optional kurze Wartezeit
    
        // 3. Über das Objekt fahren (Z-Achse leicht anheben)
        geometry_msgs::msg::Pose lift_pose = object_pose;
        lift_pose.position.z += 0.05;  // 5 cm anheben
    
        move_group_->setPoseTarget(lift_pose);
        if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Lifting object...");
            move_group_->execute(plan);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Lift motion failed.");
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
        // Warten, bis jemand den Topic abonniert hat
        while (request_joint_state->get_subscription_count() < 1 && rclcpp::ok()) {
            RCLCPP_WARN(this->get_logger(), "Waiting for subscriber to /request_joint_states...");
            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }

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
