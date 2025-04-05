// TurtleControllerNode

#include "rclcpp/rclcpp.hpp"

#include "custom_turtle_interfaces/srv/catch_turtle.hpp"
#include "custom_turtle_interfaces/msg/turtle.hpp"
#include "custom_turtle_interfaces/msg/turtle_array.hpp"

#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <exception>
#include <limits>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>


constexpr double SMALL_NUMBER = 1e-6;


// PID Controller Implementation
// Definition: The PID controller computes a control signal based on the error between a desired setpoint and the current state.
// The formula in discrete form is:
// Output = k_p * error + k_i * (integral + error * delta_time) + k_d * (error - prev_error) / delta_time
// Where:
// - k_p: Proportional gain, scales the current error.
// - k_i: Integral gain, accumulates past errors over time.
// - k_d: Derivative gain, predicts future errors based on the rate of change.
// - error: Current error value (e.g., distance or angle difference).
// - delta_time: Time step between control loop iterations.
// - integral: Accumulated error over time.
// - prev_error: Error value from the previous control loop iteration.

///////////////////////////////////////////////////////////
// PIDController
///////////////////////////////////////////////////////////

/**
 * @class PIDController
 * @brief Implements a discrete-time PID controller.
 * @details The PID controller calculates an output signal based on three terms:
 *          Proportional (P), Integral (I), and Derivative (D). This helps in controlling
 *          a process by adjusting the output to minimize the error between a target setpoint
 *          and the current state.
 */
class PIDController {
public:
    // 1) Constructor
    /**
     * @brief Constructs a PIDController with the given gains and settings.
     * @param kp Proportional gain.
     * @param ki Integral gain.
     * @param kd Derivative gain.
     * @param max_integral The maximum absolute value of the integral term.
     * @param type A descriptive string for the PID controller (e.g., "Linear" or "Angular").
     */
    PIDController(double kp, double ki, double kd, double max_integral = 20.0, std::string type = "general_pid", int loggerlv = 1)
        : kp_(kp), ki_(ki), kd_(kd), integral_(0.0), prev_error_(0.0), max_integral_(max_integral),
          prev_integral_(0.0), stagnation_count_(0), stagnation_threshold_(3), type_(type), loggerlv_(loggerlv) {}

    // 2) Compute method
    /**
     * @brief Computes the PID controller output based on the current error and elapsed time.
     * @param error The current error value (difference between setpoint and measured value).
     * @param delta_time The time interval since the last compute call.
     * @return The calculated control output.
     */
    double compute(double error, double delta_time) {
        // Proportional term
        double proportional = kp_ * error;

        // Integral term
        integral_ += error * delta_time;

        // Clamp integral to prevent windup (i.e. setting minimum and maximum integral to avoid overshooting the target)
        integral_ = std::clamp(integral_, -max_integral_, max_integral_);

        // Check for integral stagnation (i.e. if the integral has reached its maximum for more than a few cycles)
        if (loggerlv_ > 0) {RCLCPP_INFO(rclcpp::get_logger("pid_controller"), "std::abs(integral_ - prev_integral_) %.3f", std::abs(integral_ - prev_integral_) );}
        if (std::abs(integral_ - prev_integral_) < SMALL_NUMBER) {
            stagnation_count_++;
            if (stagnation_count_ >= stagnation_threshold_) {
                integral_ = 0.0; // Reset integral
                stagnation_count_ = 0; // Reset stagnation counters
                if (loggerlv_ > 0) {RCLCPP_INFO(rclcpp::get_logger("pid_controller"), "Integral term reset due to stagnation");}
            }
        } else {
            stagnation_count_ = 0; // Reset if integral is changing
        }

        prev_integral_ = integral_; // Update previous integral value

        // Derivative term
        double derivative = -1;
        if (delta_time <= SMALL_NUMBER){
            derivative = error - prev_error_;
        } else {
            derivative = (error - prev_error_) / delta_time;
        }

        // PID output
        double output = proportional + (ki_ * integral_) + (kd_ * derivative);

        // Update error for next iteration
        prev_error_ = error;

        // Logging
        if (loggerlv_ > 0) {
            RCLCPP_INFO(rclcpp::get_logger("pid_controller"),
                        "For PID - %s: Error: %.3f, Delta Time: %.3f, Proportional: %.3f, Integral: %.3f, Previous Integral: %.3f, Derivative: %.3f, Output: %.3f, Staggnation count: %i",
                        type_.c_str(), error, delta_time, proportional, integral_, prev_integral_, derivative, output, stagnation_count_);
        }
        return output;
    }

private:
    double kp_, ki_, kd_;         // PID gains
    double integral_;             // Accumulated error
    double prev_error_;           // Previous error for derivative
    double max_integral_;         // Max allowed integral value
    double prev_integral_;        // Previous integral value for stagnation detection
    int stagnation_count_;        // Counts how long the integral has stagnated
    int stagnation_threshold_;    // Threshold for resetting the integral term
    std::string type_;            // A string to describe the type of pid controller
    unsigned int loggerlv_;                // An integer to controller logging behaviour
    
};


///////////////////////////////////////////////////////////
// TurtleControllerNode
///////////////////////////////////////////////////////////
/**
 * @class TurtleControllerNode
 * @brief A ROS 2 node that controls a \"hunter\" turtle to chase other turtles.
 * @details Utilizes PID controllers for linear and angular velocities, listens to turtle poses,
 *          and calls a service to catch the target turtle once close enough.
 */
class TurtleControllerNode : public rclcpp::Node
{
public:
    /**
     * @brief Constructs the TurtleControllerNode and initializes parameters, subscribers, publishers, and timers.
     */
    TurtleControllerNode() : Node("turtle_controller"),
                             subscriber_first_call_(true),
                             control_loop_first_call_(true),
                             turtle_velocity_{},
                             iteration_n_(0),
                             is_PublishHuntedTargetTurtle_thread_running_(false),
                             is_target_turtle_caught_(false)
    {
        // Declare and retrieve target coordinates
        this->declare_parameter("target_xy", std::vector<double>{8.0, 8.0});
        initial_target_coords_ = this->get_parameter("target_xy").as_double_array();

        // Declare and retrieve control loop frequency
        this->declare_parameter("control_loop_frequency", static_cast<int>(20));
        control_loop_frequency_ = this->get_parameter("control_loop_frequency").as_int();

        // Declare and retrieve velocity PID parameters
        this->declare_parameter("vel_kp_ki_kd", std::vector<double>{0.5, 0.1, 0.05});
        vel_kp_ki_kd_ = this->get_parameter("vel_kp_ki_kd").as_double_array();

        // Declare and retrieve angular PID parameters
        this->declare_parameter("ang_kp_ki_kd", std::vector<double>{2.0, 0.05, 0.02});
        ang_kp_ki_kd_ = this->get_parameter("ang_kp_ki_kd").as_double_array();

        // Declare and retrieve velocity max integral value
        this->declare_parameter("vel_max_integral", 2.0);
        vel_max_integral_ = this->get_parameter("vel_max_integral").as_double();

        // Declare and retrieve velocity max integral value
        this->declare_parameter("ang_max_integral", 2.0);
        ang_max_integral_ = this->get_parameter("ang_max_integral").as_double();

        // Declare and retrieve minimum distance to consider target reached
        this->declare_parameter("min_distance_to_reach", 0.1);
        minDistanceToReach_ = this->get_parameter("min_distance_to_reach").as_double();

        // Declare and retrieve distance threshold used to judge if the turtle will turn before moving
        this->declare_parameter("distance_threshold", double(0.1));
        distance_threshold_ = this->get_parameter("distance_threshold").as_double();

        // Declare and retrieve the turn factor threshold
        this->declare_parameter("turn_factor_threshold", double(1.0));
        turn_factor_threshold_ = this->get_parameter("turn_factor_threshold").as_double();

        // Declare and retrieve debug option
        this->declare_parameter("debug", 1);
        debug_ = this->get_parameter("debug").as_int();

        // Declare and retrive logger level option
        this->declare_parameter("loggerlv", 1);
        loggerlv_ = this->get_parameter("loggerlv").as_int();

        if (debug_ > 3){
            alive_turtles_ = this->debugReturnTurtleArray();
        }

        setTarget(initial_target_coords_[0], initial_target_coords_[1]);
        init_turtle(turtle_hunter_);
        init_turtle(turtle_target_);

        linear_pid_ = std::make_shared<PIDController>(
            vel_kp_ki_kd_[0], vel_kp_ki_kd_[1], vel_kp_ki_kd_[2], vel_max_integral_, "Linear", loggerlv_
        );
        angular_pid_ = std::make_shared<PIDController>(
            ang_kp_ki_kd_[0], ang_kp_ki_kd_[1], ang_kp_ki_kd_[2], ang_max_integral_, "Angular", loggerlv_
        );

        subscriber_ = this->create_subscription<turtlesim::msg::Pose>(
            "turtle1/pose",
            10,
            std::bind(&TurtleControllerNode::callbackTurtlePoseSubscriber, this, std::placeholders::_1));

        alive_turtles_subscriber_ = this->create_subscription<custom_turtle_interfaces::msg::TurtleArray>(
            "alive_turtles",
            10,
            std::bind(&TurtleControllerNode::callbackAliveTurtleSubscriber, this, std::placeholders::_1));

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(control_loop_frequency_),
            std::bind(&TurtleControllerNode::controlLoop, this));

        if (loggerlv_ > 0) {RCLCPP_INFO(this->get_logger(), "TurtleControllerNode started");}
    }

private:
    // 1) Main control loop
        /**
     * @brief Main control loop that calculates and publishes velocity commands to chase turtles.
     * @details Determines the closest turtle to chase, uses PID controllers to generate velocity commands,
     *          and calls a service to catch the turtle if reached.
     */
    void controlLoop()
    {
        if (loggerlv_ > 0) {RCLCPP_INFO(this->get_logger(), "%i miliseconds have passed", control_loop_frequency_);}
        printAliveTurtles(alive_turtles_);

        if (control_loop_first_call_){
            if (loggerlv_ > 0) {RCLCPP_INFO(this->get_logger(), "Control loop has been started");}
            control_loop_first_call_=false;
        }

        if (!alive_turtles_.turtles.empty() && turtle_hunter_.name != "") {
            // This control loop only runs if there are turtles to hunt AND if the turtle_hunter has been established
            if (turtle_target_.name == "") {
                // For initial setup for first ever target turtle
                setTargetTurtle(alive_turtles_.turtles[calculateClosestTurtle()]);
            } else {
                // After the first target has been set
                printTurtlePose(turtle_hunter_);
                printTurtlePose(turtle_target_);

                double distance = calculateDistance(turtle_hunter_, turtle_target_);
                double angular_error = calculateAngularError(turtle_hunter_, turtle_target_);
                bool target_reached = isTargetReached(distance);

                if (!target_reached) {
                    // Move the hunter turtle towards the target
                    computeVelocityCommands(distance, angular_error);
                    publisher_->publish(turtle_velocity_);
                } else {
                    // If the target is reached
                    if (!is_target_turtle_caught_.load()) {
                        if (!is_PublishHuntedTargetTurtle_thread_running_.exchange(true)) {
                            std::thread(&TurtleControllerNode::callPublishTargetTurtleCaught, this, turtle_target_).detach();
                        }
                    } else {
                        setTargetTurtle(alive_turtles_.turtles[calculateClosestTurtle()]);
                        is_target_turtle_caught_.store(false);
                    }
                }
            }
        }
        iteration_n_++;
    }

    // 2) Callback: Pose subscriber
    /**
     * @brief Callback function for the turtle1/pose subscription.
     * @param msg Shared pointer to the received Pose message.
     */
    void callbackTurtlePoseSubscriber(const turtlesim::msg::Pose::SharedPtr msg){
        current_coords_ = *msg;
        turtle_hunter_.name = "turtle1";
        poseIntoTurtle(turtle_hunter_, *msg);

        if (subscriber_first_call_){
            printTurtlePose(turtle_hunter_);
            if (loggerlv_ > 0) {RCLCPP_INFO(this->get_logger(), "callbackTurtlePoseSubscriber was called - First Time");}
            subscriber_first_call_=false;
        }
    }

    // 3) Callback: Alive turtle subscriber
    /**
     * @brief Callback function for the alive_turtles subscription.
     * @param msg The received TurtleArray message containing currently alive turtles.
     */
    void callbackAliveTurtleSubscriber(const custom_turtle_interfaces::msg::TurtleArray msg){
        alive_turtles_ = msg;
    }

    // 4) ROS call method: publish target turtle caught
    /**
     * @brief Calls the "catch_turtle" service to remove the hunted target turtle.
     * @param hunted_target_turtle The turtle that is to be caught.
     * @details This function waits for the service to become available, sends a request,
     *          and updates internal flags based on the service response.
     */
    void callPublishTargetTurtleCaught(custom_turtle_interfaces::msg::Turtle hunted_target_turtle){
        try {
            if (loggerlv_ > 0) {RCLCPP_INFO(this->get_logger(), "callPublishTargetTurtleCaught function called");}
            auto client = this->create_client<custom_turtle_interfaces::srv::CatchTurtle>("catch_turtle");

            while(!client->wait_for_service(std::chrono::seconds(1))){
                if (loggerlv_ > 0) {RCLCPP_WARN(this->get_logger(), "Waiting for the turtle_spawner server to be up and advertise the 'catch_turtle' service");}
            }

            auto catch_request = std::make_shared<custom_turtle_interfaces::srv::CatchTurtle::Request>();
            catch_request->name = hunted_target_turtle.name;
            auto future = client->async_send_request(catch_request);

            try {
                auto response = future.get();
                if (response->success){
                    if (loggerlv_ > 0) {RCLCPP_INFO(this->get_logger(),"The turtle %s was caught and killed", hunted_target_turtle.name.c_str());}
                    is_target_turtle_caught_.store(true);
                } else {
                    if (loggerlv_ > 0) {RCLCPP_INFO(this->get_logger(),"The turtle %s was caught but not killed", hunted_target_turtle.name.c_str());}
                    is_target_turtle_caught_.store(false);
                }
            } catch (const std::exception &e) {
                if (loggerlv_ > 0) {RCLCPP_ERROR(this->get_logger(), "Error processing service response: %s", e.what());}
                is_target_turtle_caught_.store(false);
            }

            is_PublishHuntedTargetTurtle_thread_running_.store(false);

        } catch (const std::exception &e) {
            if (loggerlv_ > 0) {RCLCPP_ERROR(this->get_logger(), "Service call 'catch_turtle' failed");}
            is_target_turtle_caught_.store(false);
            is_PublishHuntedTargetTurtle_thread_running_.store(false);
        }
    }

    // 5) Core routine: set target turtle
    /**
     * @brief Sets the turtle to be pursued by the hunter turtle.
     * @param target_turtle The turtle that will become the new target.
     */
    void setTargetTurtle(custom_turtle_interfaces::msg::Turtle target_turtle){
        turtle_target_ = std::move(target_turtle);
    }

    // 6) Core routine: compute velocity commands
    /**
     * @brief Computes and assigns velocity commands to chase the target turtle.
     * @param distance The distance between the hunter turtle and the target turtle (meters).
     * @param angular_error The angular difference (radians) between the hunter turtle's heading and the target.
     */
    void computeVelocityCommands(double distance, double angular_error) {
        double linear_velocity = 0;
        double angular_velocity = 0;

        if (iteration_n_==0) {
            linear_velocity = linear_pid_->compute(distance, SMALL_NUMBER);
            angular_velocity = angular_pid_->compute(angular_error, SMALL_NUMBER);
            previous_time_ = std::chrono::system_clock::now();
            if (loggerlv_ > 0) {RCLCPP_INFO(this->get_logger(), "The initial time delta is %f", SMALL_NUMBER);}
        }
        else {
            auto currentTime = std::chrono::system_clock::now();
            std::chrono::duration<float> time_difference = currentTime - previous_time_;
            linear_velocity = linear_pid_->compute(distance, time_difference.count());
            angular_velocity = angular_pid_->compute(angular_error, time_difference.count());
            previous_time_ = currentTime;
            if (loggerlv_ > 0) {RCLCPP_INFO(this->get_logger(), "The time delta is %f", time_difference.count());}
        }

        double turn_factor = std::abs(angular_error) / std::max(static_cast<double>(distance), 0.01);

        if (isTargetReached(distance)) {
            turtle_velocity_.linear.x = 0.0;
            turtle_velocity_.angular.z = 0.0;
        } else if (turn_factor > turn_factor_threshold_ && distance > distance_threshold_) {
            turtle_velocity_.linear.x = 0.0;
            turtle_velocity_.angular.z = angular_velocity;
        } else {
            turtle_velocity_.linear.x = linear_velocity;
            turtle_velocity_.angular.z = angular_velocity;
        }

        if (loggerlv_ > 0) {
            RCLCPP_INFO(this->get_logger(),
                        "Linear Velocity: %.2f, Angular Velocity: %.2f, Distance: %.2f, Angular Error: %.2f, Turn Factor: %.2f",
                        turtle_velocity_.linear.x, turtle_velocity_.angular.z, distance, angular_error, turn_factor);
        }
    }

    // 7) Utility function: is target reached?
    /**
     * @brief Checks if the hunter turtle has reached the target (within a minimum distance).
     * @param distance The distance between the hunter turtle and the target turtle.
     * @return True if the distance is less than or equal to the configured threshold.
     */
    bool isTargetReached(double distance){
        return distance <= minDistanceToReach_;
    }

    // 8) Utility function: set target
    /**
     * @brief Sets an (x, y) target coordinate for debugging or alternative use.
     * @param x The x-coordinate of the target.
     * @param y The y-coordinate of the target.
     */
    void setTarget(float x, float y){
        target_coords_.x = x;
        target_coords_.y = y;
    }

    // 9) Utility function: calculate closest turtle
    /**
     * @brief Calculates the index of the closest turtle in the alive_turtles_ array.
     * @return The index of the closest turtle. Returns -1 if no turtles are available.
     */
    int calculateClosestTurtle(){
        int closest_turtle_index = -1;
        double min_distance = std::numeric_limits<double>::max();

        for (int i = 0; i < static_cast<int>(alive_turtles_.turtles.size()); i++){
            auto turtle_target = alive_turtles_.turtles[i];
            double min_distance_tmp = calculateDistance(turtle_hunter_, turtle_target);
            if (min_distance_tmp < min_distance) {
                min_distance = min_distance_tmp;
                closest_turtle_index = i;
            }
        }
        return closest_turtle_index;
    }

    // 10) Utility function: calculate distance
    /**
     * @brief Calculates the distance between two turtles.
     * @param turtle_hunter The hunter turtle.
     * @param turtle_target The target turtle.
     * @return The Euclidean distance between the two turtles.
     */
    double calculateDistance(custom_turtle_interfaces::msg::Turtle& turtle_hunter, custom_turtle_interfaces::msg::Turtle& turtle_target){
        return std::hypot(turtle_target.x - turtle_hunter.x, turtle_target.y - turtle_hunter.y);
    }

    // 11) Utility function: calculate angular error
    /**
     * @brief Calculates the angular error between the hunter turtle's heading and the line to the target.
     * @param turtle_hunter The hunter turtle.
     * @param turtle_target The target turtle.
     * @return The angular difference in radians, normalized to the range [-pi, pi].
     */
    double calculateAngularError(custom_turtle_interfaces::msg::Turtle& turtle_hunter, custom_turtle_interfaces::msg::Turtle& turtle_target){
        double theta_desired = std::atan2(turtle_target.y - turtle_hunter.y, turtle_target.x - turtle_hunter.x);
        double theta_diff = theta_desired - turtle_hunter.theta;
        theta_diff = std::fmod(theta_diff + M_PI, 2 * M_PI) - M_PI;
        return theta_diff;
    }

    // 12) Utility function: pose into turtle
    /**
     * @brief Converts a turtlesim::msg::Pose into a custom_turtle_interfaces::msg::Turtle.
     * @param turtle The turtle msg to fill.
     * @param pose The input msg pose, from turtlesim.
     */
    void poseIntoTurtle(custom_turtle_interfaces::msg::Turtle& turtle, turtlesim::msg::Pose& pose){
        turtle.x = pose.x;
        turtle.y = pose.y;
        turtle.theta = pose.theta;
        turtle.linear_velocity = pose.linear_velocity;
        turtle.angular_velocity = pose.angular_velocity;
    }

    // 13) Utility function: init turtle
    /**
     * @brief Initializes a custom_turtle_interfaces::msg::Turtle struct with default values.
     * @param turtle The turtle struct to initialize.
     */
    void init_turtle(custom_turtle_interfaces::msg::Turtle& turtle){
        turtle.name = "";
        turtle.x = 0.0;
        turtle.y = 0.0;
        turtle.theta = 0.0;
        turtle.linear_velocity = 0.0;
        turtle.angular_velocity = 0.0;
    }

    // 14) Debug: print turtle pose
    /**
     * @brief Logs the pose of a given turtle.
     * @param turtle The turtle whose pose will be logged.
     */
    void printTurtlePose(custom_turtle_interfaces::msg::Turtle& turtle){
        if (loggerlv_ > 0) {
            RCLCPP_INFO(this->get_logger(),
                "The pose of the turtle '%s'  is: \\n x=%f, \\n y=%f \\n theta=%f",
                turtle.name.c_str(), turtle.x, turtle.y, turtle.theta);
        }
    }

    // 15) Debug: print target turtle pose
    /**
     * @brief Logs the pose of a target turtle.
     * @param target_turtle The target turtle to log.
     */
    void printTargetTurtlePose(custom_turtle_interfaces::msg::Turtle target_turtle){
        if (loggerlv_ > 0) {
            RCLCPP_INFO(this->get_logger(),
                "The name of the target turlte is: %s\\nThe pose of turtle1 is: \\n x=%f, \\n y=%f",
                target_turtle.name.c_str(), target_turtle.x, target_turtle.y);
        }
    }

    // 16) Debug: print alive turtles
    /**
     * @brief Logs the names and positions of all currently alive turtles.
     * @param AliveTurtles A reference to the TurtleArray containing all alive turtles.
     */
    void printAliveTurtles(custom_turtle_interfaces::msg::TurtleArray& AliveTurtles){
        if (loggerlv_ > 0) {
            RCLCPP_INFO(this->get_logger(), "Alive Turtles");
            for (unsigned int i = 0; i < AliveTurtles.turtles.size(); i++){
                auto turtle = AliveTurtles.turtles[i];
                RCLCPP_INFO(this->get_logger(), "%s: [%f, %f]", turtle.name.c_str(), turtle.x, turtle.y);
            }
        }
    }

    // 17) Debug: return a mock turtle array
    /**
     * @brief Returns a mock TurtleArray for debugging.
     * @return A TurtleArray populated with dummy turtles.
     */
    custom_turtle_interfaces::msg::TurtleArray debugReturnTurtleArray(){
        struct debugTurtle{
            // A structure to simulate the Turtle.msg
            std::string name;
            double x;
            double y;
            double theta;
        };

        struct debugTurtleArray{
            // A structure to simulate the TurtleArray.msg
            std::vector<debugTurtle> turtles_vector;
        };

        debugTurtleArray turtle_array_struct = {
            {
                {"TargetTurtle1", 1.0, 1.0, 0.0},
                {"TargetTurtle2", 2.5, 3.5, M_PI / 4},
                {"TargetTurtle3", 4.0, 5.5, M_PI / 2},
                {"TargetTurtle4", 6.0, 2.0, M_PI},
                {"TargetTurtle5", 7.5, 7.5, 3 * M_PI / 2},
                {"TargetTurtle6", 10.0, 3.0, 2 * M_PI},
                {"TargetTurtle7", 8.0, 9.0, M_PI / 6},
                {"TargetTurtle8", 5.0, 4.5, M_PI / 3},
                {"TargetTurtle9", 3.0, 7.0, 5 * M_PI / 4},
                {"TargetTurtle10", 9.0, 1.0, 7 * M_PI / 4}
            }
        };

        custom_turtle_interfaces::msg::TurtleArray turtle_array;
        turtle_array.turtles.resize(turtle_array_struct.turtles_vector.size());

        for (int i = 0; i < static_cast<int>(turtle_array.turtles.size()); i++){
            auto *msgturtle = &turtle_array.turtles[i];
            auto *vecturtle = &turtle_array_struct.turtles_vector[i];

            msgturtle->name = vecturtle->name;
            msgturtle->x = vecturtle->x;
            msgturtle->y = vecturtle->y;
            msgturtle->theta = vecturtle->theta;
        }
        return turtle_array;
    }

    // 18) Private variables
    std::unordered_map<std::string, std::unordered_map<std::string, float>> turtles_poses_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<double> initial_target_coords_;
    int control_loop_frequency_;
    bool subscriber_first_call_;
    bool control_loop_first_call_;
    std::vector<double> vel_kp_ki_kd_;
    std::vector<double> ang_kp_ki_kd_;
    double vel_max_integral_;
    double ang_max_integral_;

    geometry_msgs::msg::Twist turtle_velocity_;
    double minDistanceToReach_;

    turtlesim::msg::Pose current_coords_;
    turtlesim::msg::Pose target_coords_;

    std::shared_ptr<PIDController> linear_pid_;
    std::shared_ptr<PIDController> angular_pid_;

    double distance_threshold_;
    double turn_factor_threshold_;

    int64_t iteration_n_;
    std::chrono::system_clock::time_point previous_time_;

    unsigned int debug_;
    unsigned int loggerlv_;

    custom_turtle_interfaces::msg::TurtleArray alive_turtles_;

    custom_turtle_interfaces::msg::Turtle turtle_hunter_;
    custom_turtle_interfaces::msg::Turtle turtle_target_;
    rclcpp::Subscription<custom_turtle_interfaces::msg::TurtleArray>::SharedPtr alive_turtles_subscriber_;

    std::atomic<bool> is_PublishHuntedTargetTurtle_thread_running_;
    std::atomic<bool> is_target_turtle_caught_;

};






int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
