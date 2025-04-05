// TurtleSpawnerNode

#include "rclcpp/rclcpp.hpp"

#include "custom_turtle_interfaces/srv/catch_turtle.hpp"
#include "custom_turtle_interfaces/msg/turtle.hpp"
#include "custom_turtle_interfaces/msg/turtle_array.hpp"

#include "turtlesim/srv/spawn.hpp"
#include "turtlesim/srv/kill.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <iterator>
#include <memory>
#include <string>
#include <thread>


///////////////////////////////////////////////////////////
// TurtleSpawnNode
///////////////////////////////////////////////////////////
/**
 * @class TurtleSpawnerNode
 * @brief A node responsible for spawning and managing turtles.
 * @details This class controls how often turtles are spawned (and can be killed) in a turtlesim environment.
 *          It offers services, publishes alive turtles, and handles kill requests.
 */
using std::placeholders::_1;
using std::placeholders::_2;


class TurtleSpawnerNode  : public rclcpp::Node
{
public:
    /**
     * @brief Constructs the TurtleSpawnerNode and initializes parameters, services, timers, and publishers.
     */
    TurtleSpawnerNode () : Node("turtle_spawner"),
                           iteration_n_{0},
                           seed_(42),
                           spawn_turtle_counter_(0),
                           current_number_of_turtles_(0),
                           previous_time_(std::chrono::steady_clock::now())
    {
        // Declare and retrieve control loop frequency in milliseconds
        this->declare_parameter("control_loop_frequency", 20);
        control_loop_frequency_ = this->get_parameter("control_loop_frequency").as_int();

        // Delcare and retrieve turle spawn frequency in milliseconds
        this->declare_parameter("turtle_spawn_frequency", 2000);
        spawn_frequency_ = this->get_parameter("turtle_spawn_frequency").as_int();

        // Declare and retrieve maximum number of turtles that can be spawned
        this->declare_parameter("max_turtles", 5);
        max_turtles_ = this->get_parameter("max_turtles").as_int();

        // Declare and retrieve debug option
        this->declare_parameter("seed", 42);
        seed_ = this->get_parameter("seed").as_int();

        // Declare and retrieve debug option
        this->declare_parameter("debug", 1);
        debug_ = this->get_parameter("debug").as_int();

        // Declare and retrive logger level option
        this->declare_parameter("loggerlv", 1);
        loggerlv_ = this->get_parameter("loggerlv").as_int();

        if (loggerlv_ > 0) {RCLCPP_INFO(this->get_logger(), "TurtleSpawnerNode has been started");}

        // std::srand(static_cast<unsigned>(std::time(nullptr)));
        std::srand(seed_);

        publisher_ = this->create_publisher<custom_turtle_interfaces::msg::TurtleArray>(
            "alive_turtles",
             10);

        server_ = this->create_service<custom_turtle_interfaces::srv::CatchTurtle>(
            "catch_turtle",
            std::bind(&TurtleSpawnerNode::callbackCatchTurtle, this, _1, _2));

        timer_ = this->create_wall_timer(std::chrono::milliseconds(control_loop_frequency_),
                                        std::bind(&TurtleSpawnerNode::controlLoop, this));
    }

private:
    // 1) Main control loop: controls how often a turtle respawns
    /**
     * @brief Main control loop that spawns new turtles at a certain frequency and publishes alive turtles.
     * @details Checks if the spawn interval has elapsed, spawns a new turtle if under the maximum limit,
     *          and kills a debug turtle if in high debug mode. Publishes the list of alive turtles.
     */
    void controlLoop(){
        if (iteration_n_ == 0){
            if (loggerlv_ > 0) {RCLCPP_INFO(this->get_logger(), "Control loop has been started");}
        }
        if (loggerlv_ > 0) {RCLCPP_INFO(this->get_logger(), "%i miliseconds have passed", static_cast<int>(control_loop_frequency_));}

        // Check if trutle spawn time has elapsed and print alive turtles
        if (this->haveMillisecondsElapsed(spawn_frequency_)){
            // debug
            if (debug_>3){
                if(iteration_n_ != 0){
                    this->killTurtleRoutine(debug_past_turtle_);
                }
            }

            if (current_number_of_turtles_ < max_turtles_) {
                this->spawnTurtleRoutine();
            }

            if (!alive_turtles_.turtles.empty()){
                if (loggerlv_ > 0) {
                    for (unsigned i = 0; i < alive_turtles_.turtles.size(); i++){
                        RCLCPP_INFO(this->get_logger(), "%s", alive_turtles_.turtles[i].name.c_str());
                        RCLCPP_INFO(this->get_logger(), "%i miliseconds have passed", static_cast<int>(spawn_frequency_));
                    }
                }
            }

        }

        this->publishAliveTurtles();
        iteration_n_++;
    }

    // 2) Service callback: handle requests to catch/kill turtles
    /**
     * @brief Service callback for the "catch_turtle" service.
     * @param request  Shared pointer to the incoming request, which includes the name of the turtle to catch.
     * @param response Shared pointer to the service response, indicating success or failure.
     */
    void callbackCatchTurtle(const custom_turtle_interfaces::srv::CatchTurtle::Request::SharedPtr request,
                             const custom_turtle_interfaces::srv::CatchTurtle::Response::SharedPtr response){

        std::string turtle_name = request->name;

        if (loggerlv_ > 0) {RCLCPP_INFO(this->get_logger(), "callbackCatchTurtle function called. Turtle caught: %s", turtle_name.c_str());}

        bool success = false;

        auto turtle_to_kill = this -> getTurtleFromTurtleArray(alive_turtles_, turtle_name);

        success = this->killTurtleRoutine(turtle_to_kill);

        response->success = success;

        if (loggerlv_ > 0) {RCLCPP_INFO(this->get_logger(), "Success of turtle '%s' removal: %s", turtle_name.c_str(), success? "True" : "False");}
    }

    // 3) Core routine: spawning a turtle
    /**
     * @brief Spawns a new turtle by adding it to the local list and then calling the /spawn service in a separate thread.
     * @return True if the turtle was successfully added to the list, False otherwise.
     */
    bool spawnTurtleRoutine(){
        auto turtle_target = getRandomTurtle();
        bool success = this->addTurtleToList(alive_turtles_, turtle_target);
        if(success){
            std::thread turtle_spawner_thread (std::bind(&TurtleSpawnerNode::callSpawnTurtle, this, turtle_target));
            turtle_spawner_thread.detach();
            current_number_of_turtles_++;
            spawn_turtle_counter_++;
            if (loggerlv_ > 0) {RCLCPP_WARN(this->get_logger(), "Turtle %s spawned at location [%f, %f]", turtle_target.name.c_str(), turtle_target.x, turtle_target.y);}
            debug_past_turtle_ = turtle_target;
        } else {
           if (loggerlv_ > 0) {RCLCPP_WARN(this->get_logger(), "Turtle %s could not be spawned at location [%f, %f]", turtle_target.name.c_str(), turtle_target.x, turtle_target.y);}
        }
        return success;
    }

    // 4) Core routine: killing a turtle
    /**
     * @brief Kills the specified turtle by removing it from the list and calling the /kill service.
     * @param turtle Reference to the turtle to be killed.
     * @return True if the turtle was successfully removed, False otherwise.
     */
    bool killTurtleRoutine(custom_turtle_interfaces::msg::Turtle& turtle){
        bool success = this->removeTurtleFromList(alive_turtles_, turtle.name);
        if (loggerlv_ > 0) {RCLCPP_INFO(this->get_logger(), "Success: %s", success? "True" : "False");}
        if (success) {
            std::thread turtle_killer_thread (std::bind(&TurtleSpawnerNode::callKillTurtle, this, turtle));
            turtle_killer_thread.detach();
            current_number_of_turtles_-- ;
            if (loggerlv_ > 0) {RCLCPP_INFO(this->get_logger(), "Turtle %s at location [%f, %f] killed.", turtle.name.c_str(), turtle.x, turtle.y);}
        } else {
            if (loggerlv_ > 0) {RCLCPP_WARN(this->get_logger(), "Turtle %s at location [%f, %f] could not be killed.", turtle.name.c_str(),turtle.x, turtle.y);}
        }
        return (success);
    }

    // 5) ROS call method: call the /kill service
    /**
     * @brief Calls the /kill service to remove a turtle from the turtlesim.
     * @param turtle The turtle to be killed.
     */
    void callKillTurtle(custom_turtle_interfaces::msg::Turtle turtle){
        if (loggerlv_ > 0) {RCLCPP_INFO(this->get_logger(), "callKillTurtle function called");}

        auto client = this->create_client<turtlesim::srv::Kill>("/kill");

        while (!client->wait_for_service(std::chrono::seconds(1))){
            if (loggerlv_ > 0) {RCLCPP_WARN(this->get_logger(), "Waiting for turtlesim server to be up and adertise the '/kill' service");}
        }

       auto kill_request = createKillRequestFromTurtle(turtle);
       auto future =client->async_send_request(kill_request);

       try{
           future.get();
           if (loggerlv_ > 0) {RCLCPP_INFO(this->get_logger(), "The trutle %s was killed", kill_request->name.c_str());}

       }catch (const std::exception &e){
           if (loggerlv_ > 0) {RCLCPP_ERROR(this->get_logger(), "Service call '/kill' failed");}
       }
    }

    // 6) ROS call method: call the /spawn service
    /**
     * @brief Calls the /spawn service to add a new turtle in the turtlesim.
     * @param turtle The turtle to be spawned.
     */
    void callSpawnTurtle(custom_turtle_interfaces::msg::Turtle turtle)
    {
        if (loggerlv_ > 0) {RCLCPP_INFO(this->get_logger(), "callSpawnTurtle function called");}

        auto client = this->create_client<turtlesim::srv::Spawn>("/spawn");

        while (!client->wait_for_service(std::chrono::seconds(1))){
            if (loggerlv_ > 0) {RCLCPP_WARN(this->get_logger(), "Waiting for turtlesim server to be up and advertise the '/spawn' service");}
        }

        auto spawn_request = createSpawnRequestFromTurtle(turtle);
        auto future = client->async_send_request(spawn_request);

        try{
            auto response = future.get();
            if (loggerlv_ > 0) {
                RCLCPP_INFO(this->get_logger(), "The turtle %s was spawned in location [%f, %f]",
                                                    response->name.c_str(), spawn_request->x, spawn_request->y);
            }
        } catch (const std::exception &e) {
            if (loggerlv_ > 0) {RCLCPP_ERROR(this->get_logger(), "Service call '/spawn' failed: %s", e.what());}
        }
    }

    // 7) Function to determine if enough time (milliseconds) has elapsed
    /**
     * @brief Determines if the specified number of milliseconds have elapsed since last check.
     * @param milliseconds_threshold The time threshold in milliseconds.
     * @return True if the time threshold has been exceeded, False otherwise.
     */
    bool haveMillisecondsElapsed(int milliseconds_threshold){
        auto now_time = std::chrono::steady_clock::now();
        long time_difference = std::chrono::duration_cast<std::chrono::milliseconds>(now_time - previous_time_).count();
        bool time_passed = false;

        if (time_difference > milliseconds_threshold){
            previous_time_ = now_time;
            time_passed = true;
        }
        return time_passed;
    }

    // 8) Publisher to list currently alive turtles
    /**
     * @brief Publishes the current list of alive turtles to the "alive_turtles" topic.
     */
    void publishAliveTurtles(){
        custom_turtle_interfaces::msg::TurtleArray tmp_alive_turtles = alive_turtles_;
        publisher_->publish(tmp_alive_turtles);
    }

    // 9) Get a turtle object from the array by name
    /**
     * @brief Retrieves a turtle from the specified TurtleArray by name.
     * @param turtle_array Reference to the TurtleArray in which to search.
     * @param turtle_name The name of the turtle to retrieve.
     * @return A copy of the found turtle, or an empty turtle if not found.
     */
    custom_turtle_interfaces::msg::Turtle getTurtleFromTurtleArray(custom_turtle_interfaces::msg::TurtleArray& turtle_array, std::string& turtle_name){
        // int turtle_index = -1;
        for (int i = 0; i < static_cast<int>(turtle_array.turtles.size()); i++){
            auto current_turtle =  turtle_array.turtles[i];
            if (current_turtle.name == turtle_name){
                // turtle_index = i;
                return current_turtle;
            }
        }
        custom_turtle_interfaces::msg::Turtle empty_turtle{};
        return empty_turtle;
    }

    // 10) Create the /spawn request from a Turtle message
    /**
     * @brief Creates a request for the /spawn service from a given turtle's parameters.
     * @param turtle The turtle from which to create the spawn request.
     * @return A shared pointer to the Spawn::Request object.
     */
    turtlesim::srv::Spawn::Request::SharedPtr createSpawnRequestFromTurtle(const custom_turtle_interfaces::msg::Turtle& turtle){
        auto srv_spawn = std::make_shared<turtlesim::srv::Spawn::Request>();
        srv_spawn->name = turtle.name;
        srv_spawn->x = static_cast<float>(turtle.x);
        srv_spawn->y = static_cast<float>(turtle.y);
        srv_spawn->theta = static_cast<float>(turtle.theta);
        return srv_spawn;
    }

    // 11) Create the /kill request from a Turtle message
    /**
     * @brief Creates a request for the /kill service from a given turtle's parameters.
     * @param turtle The turtle from which to create the kill request.
     * @return A shared pointer to the Kill::Request object.
     */
    turtlesim::srv::Kill::Request::SharedPtr createKillRequestFromTurtle(const custom_turtle_interfaces::msg::Turtle& turtle){
        auto srv_kill = std::make_shared<turtlesim::srv::Kill::Request>();
        srv_kill->name = turtle.name;
        return srv_kill;
    }

    // 12) Generate a random double in [min, max]
    /**
     * @brief Generates a random double within the specified range.
     * @param min The lower bound.
     * @param max The upper bound.
     * @return A random double in the range [min, max].
     */
    double getRandomNumber(double min, double max) {
        return min + ((static_cast<double>(std::rand()) / RAND_MAX) * (max - min));
    }

    // 13) Generate and return a randomly configured Turtle
    /**
     * @brief Generates a random turtle, assigning random position and orientation.
     * @return A custom_turtle_interfaces::msg::Turtle object with random values.
     */
    custom_turtle_interfaces::msg::Turtle getRandomTurtle(){
        custom_turtle_interfaces::msg::Turtle turtle;
        turtle.name = "turtle_target" + std::to_string(spawn_turtle_counter_ + 1);
        turtle.x = this->getRandomNumber(0.0, 11.0);
        turtle.y = this->getRandomNumber(0.0, 11.0);
        turtle.theta = this->getRandomNumber(-M_PI, M_PI);
        turtle.linear_velocity = 0.0;
        turtle.angular_velocity = 0.0;
        return turtle;
    }

    // 14) Function to add new turtle to the turtle array to be published
    /**
     * @brief Adds a turtle to the given TurtleArray.
     * @param turtle_array Reference to the TurtleArray to which the turtle will be added.
     * @param turtle Reference to the turtle to add.
     * @return True if the turtle was successfully added, False otherwise.
     */
    bool addTurtleToList(custom_turtle_interfaces::msg::TurtleArray& turtle_array,
                       custom_turtle_interfaces::msg::Turtle& turtle){

        bool success = false;
        try {
            turtle_array.turtles.push_back(turtle);
            success = true;
        } catch (const std::exception& e) {
            if (loggerlv_ > 0) {RCLCPP_ERROR(this->get_logger(), "Addition of turtle %s to list failed: %s", turtle.name.c_str(), e.what());}
            success = false;
        }
        return success;
    }

    // 15) Function to remove turtle from the array by name
    /**
     * @brief Removes a turtle from the given TurtleArray by name.
     * @param turtle_array Reference to the TurtleArray from which to remove the turtle.
     * @param turtle_name The name of the turtle to remove.
     * @return True if the turtle was found and removed, False otherwise.
     */
    bool removeTurtleFromList(custom_turtle_interfaces::msg::TurtleArray& turtle_array,
                      const std::string& turtle_name){
        // Lambda function to match the turtle name
        // Captures `turtle_name` by reference and checks if a turtle's name matches the given name
        auto lambdaMatchTurtleName = [&turtle_name](const custom_turtle_interfaces::msg::Turtle& turtle) {
            return turtle.name == turtle_name;};

        bool success = false;
        try {
            // Use std::remove_if to find and move all turtles that match the name to the end of the vector
            // `it` will point to the new logical end of the vector (the first "garbage" element)
            auto it = std::remove_if(turtle_array.turtles.begin(),
                                    turtle_array.turtles.end(),
                                    lambdaMatchTurtleName);

            if (it != turtle_array.turtles.end()) {
                // If there are any "garbage" elements (matching turtles), erase them from the vector
                turtle_array.turtles.erase(it, turtle_array.turtles.end());
            }

            // current_number_of_turtles_--;
            success = true;

        } catch (const std::exception &e) {
            if (loggerlv_ > 0) {RCLCPP_ERROR(this->get_logger(), "Removal of turtle %s from list failed: %s", turtle_name.c_str(), e.what());}
            success = false;
        }
        return success;
    }

    int a;
    std::vector<std::thread> thraeds_;

    int64_t control_loop_frequency_;
    int64_t spawn_frequency_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<custom_turtle_interfaces::srv::CatchTurtle>::SharedPtr server_;

    const custom_turtle_interfaces::srv::CatchTurtle::Request::SharedPtr dummy_request_;
    const custom_turtle_interfaces::srv::CatchTurtle::Response::SharedPtr dummy_response_;

    custom_turtle_interfaces::msg::TurtleArray alive_turtles_;

    int64_t iteration_n_;
    unsigned seed_;
    int64_t spawn_turtle_counter_;
    int64_t current_number_of_turtles_;

    int64_t max_turtles_;

    rclcpp::Publisher<custom_turtle_interfaces::msg::TurtleArray>::SharedPtr publisher_;
    std::chrono::steady_clock::time_point previous_time_;

    unsigned int debug_;
    unsigned int loggerlv_;

    std::string debug_past_turtle_name_{};
    custom_turtle_interfaces::msg::Turtle debug_past_turtle_{};
};





int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleSpawnerNode >();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
};
