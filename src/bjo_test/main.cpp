#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <string>
using std::string;
#include <memory>
// TIP To <b>Run</b> code, press <shortcut actionId="Run"/> or
// click the <icon src="AllIcons.Actions.Execute"/> icon in the gutter.
int main(int argc, char **argv) {
    // Initialize the ROS client library
    rclcpp::init(argc, argv);

    auto sam = "nnn";
    auto fred = std::make_shared<string>("aaaa");

    auto node = std::make_shared<rclcpp::Node>("vision_node");

    // Output a simple message
    auto lang = "C++";
    std::cout << "Hello and welcome to " << lang << "!\n";
    std::cout << "sam:  " << sam << "  fred: " << *fred << "\n";

    // Example: Process events (optional)
    rclcpp::spin(node);

    // Shutdown the ROS client library
    rclcpp::shutdown();

    return 0;
}

// TIP See CLion help at <a
// href="https://www.jetbrains.com/help/clion/">jetbrains.com/help/clion/</a>.
//  Also, you can try interactive lessons for CLion by selecting
//  'Help | Learn IDE Features' from the main menu.