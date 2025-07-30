#include <rclcpp/rclcpp.hpp>
#include <gazebo_msgs/srv/spawn_entity.hpp>
#include <random>
#include <string>
#include <sstream>

class BoxSpawner : public rclcpp::Node
{
public:
  BoxSpawner() : Node("box_spawner")
  {
    client_ = this->create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");
    while (!client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_INFO(this->get_logger(), "Waiting for /spawn_entity service...");
    }

    spawnMultipleBoxes(10);  // Spawn 10 random boxes
  }

private:
  rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr client_;
  std::default_random_engine gen_;
  std::uniform_real_distribution<double> x_dist_{-0.2, 0.2};
  std::uniform_real_distribution<double> y_dist_{-0.5, 0.5};
  std::uniform_real_distribution<double> yaw_dist_{0.0, 3.14};

  void spawnMultipleBoxes(int count)
  {
    for (int i = 0; i < count; ++i) {
      std::string color = (rand() % 2 == 0) ? "Red" : "Blue";
      std::string size_label = (rand() % 2 == 0) ? "Small" : "Large";
      spawnBox(i, color, size_label);
      rclcpp::sleep_for(std::chrono::milliseconds(300));
    }
  }

  void spawnBox(int id, const std::string &color, const std::string &size_label)
  {
    double size = (size_label == "Small") ? 0.1 : 0.2;
    double x = x_dist_(gen_);
    double y = y_dist_(gen_);
    double z = 0.9;
    double yaw = yaw_dist_(gen_);

    std::ostringstream name;
    name << color << "_" << size_label << "_" << id;

    std::ostringstream sdf;
    sdf << "<sdf version='1.6'>"
        << "<model name='" << name.str() << "'>"
        << "<static>false</static>"
        << "<pose>" << x << " " << y << " " << z << " 0 0 " << yaw << "</pose>"
        << "<link name='link'>"
        << "  <inertial><mass>1.0</mass></inertial>"
        << "  <collision name='collision'>"
        << "    <geometry><box><size>" << size << " " << size << " " << size << "</size></box></geometry>"
        << "  </collision>"
        << "  <visual name='visual'>"
        << "    <geometry><box><size>" << size << " " << size << " " << size << "</size></box></geometry>"
        << "    <material>"
        << "      <ambient>" << (color == "Red" ? "1 0 0 1" : "0 0 1 1") << "</ambient>"
        << "      <diffuse>" << (color == "Red" ? "1 0 0 1" : "0 0 1 1") << "</diffuse>"
        << "    </material>"
        << "  </visual>"
        << "</link>"
        << "</model>"
        << "</sdf>";

    auto request = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
    request->name = name.str();
    request->xml = sdf.str();
    request->robot_namespace = "";
    request->reference_frame = "world";

    auto result = client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "Spawned: %s", name.str().c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "❌ Failed to spawn: %s", name.str().c_str());
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BoxSpawner>();
  rclcpp::shutdown();
  return 0;
}
