#include <memory> // Required for std::shared_ptr
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

// Using declaration for convenience
using std::placeholders::_1;

class LidarSubscriber : public rclcpp::Node
{
public:
  // Constructor
  LidarSubscriber()
  : Node("lidar_pointcloud_subscriber") // Node name
  {
    // Define Quality of Service settings.
    // Sensor data usually uses reliable, best-effort, or sensor-specific QoS.
    // Using SensorDataQoS() is often a good default for sensor streams.
    // Alternatively, a simple depth like 10 with default reliability/durability works.
    auto qos = rclcpp::SensorDataQoS();
    // auto qos = rclcpp::QoS(10); // Alternative simple QoS

    // Create the subscription.
    // Template argument: Message type (PointCloud2)
    // Argument 1: Topic name - this MUST match what the Velodyne driver publishes.
    //             '/velodyne_points' is the common default for the velodyne_driver node.
    // Argument 2: QoS profile.
    // Argument 3: Callback function to process incoming messages. std::bind links the
    //             member function `topic_callback` to this subscriber. `_1` is a
    //             placeholder for the incoming message.
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/velodyne_points", qos, std::bind(&LidarSubscriber::topic_callback, this, _1));

    RCLCPP_INFO(this->get_logger(), "Lidar subscriber node started, listening on /velodyne_points");
  }

private:
  // Callback function definition
  void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
  {
    // This function gets called every time a new message arrives on "/velodyne_points".

    // Log basic information about the received cloud.
    // Accessing header info:
    // std::string frame_id = msg->header.frame_id;
    // rclcpp::Time timestamp = msg->header.stamp;

    // Accessing point cloud data properties:
    // uint32_t height = msg->height; // Typically 1 for unorganized point clouds from LIDAR
    // uint32_t width = msg->width;   // Number of points in the cloud
    // bool is_dense = msg->is_dense; // True if no invalid points (NaN, inf)

    // For this basic example, we just log that we received a message
    // and show the number of points.
    RCLCPP_INFO(this->get_logger(), "Received PointCloud2 message with %d points.", msg->width * msg->height);

    // ---- Potential Further Processing ----
    // If you needed to *process* the point cloud data in C++, you would do it here.
    // Examples:k
    // - Iterate through points (requires understanding the msg->data format and msg->fields)
    // - Filter the point cloud
    // - Detect objects (though DeepStream might be better suited for GPU-accelerated detection)
    // - Convert to other formats (e.g., PCL PointCloud) using ros2_pcl helpers if needed.
    // ------------------------------------
  }

  // Declare the subscription object
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

// Main function - the entry point of the program
int main(int argc, char * argv[])
{

  std::cout << "###### Running as UID: " << getuid() << ", GID: " << getgid() << " ######" << std::endl; // Add this line

  // Initialize the ROS 2 client library
  rclcpp::init(argc, argv);

  // Create an instance of the LidarSubscriber node and spin it,
  // which keeps it running and processing callbacks until shutdown (e.g., Ctrl+C).
  // std::make_shared creates a shared pointer to the node.
  rclcpp::spin(std::make_shared<LidarSubscriber>());

  // Shutdown the ROS 2 client library
  rclcpp::shutdown();
  return 0;
}
