#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo_ros/node.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <sdf/sdf.hh>
#include <random>
#include <sstream>
#include <map>

namespace gazebo
{

class BoxSpawnerPlugin : public WorldPlugin
{
public:
  void Load(physics::WorldPtr world, sdf::ElementPtr) override
  {
    world_ = world;
    ros_node_ = gazebo_ros::Node::Get();

    srv_ = ros_node_->create_service<std_srvs::srv::Trigger>(
      "spawn_box",
      std::bind(&BoxSpawnerPlugin::SpawnBoxCallback, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(ros_node_->get_logger(), " BoxSpawnerPlugin loaded and 'spawn_box' service available.");
  }

private:
  physics::WorldPtr world_;
  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_;

  std::default_random_engine gen_;
  std::uniform_real_distribution<double> x_dist_{0.703363, 0.7055};   // Adjust for conveyor area
  std::uniform_real_distribution<double> y_dist_{-1.09465, -1.054465};
  std::uniform_real_distribution<double> yaw_dist_{0.0, 3.14};

  std::map<std::string, int> color_counter_ = {{"Red", 1}, {"Blue", 1}};

  // Configurable size/mass values
  double small_size_ = 0.12;
  double large_size_ = 0.15;
  double small_mass_ = 2.0;
  double large_mass_ = 3.0;

  void SpawnBoxCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr res)
  {
    // Randomly assign box type
    std::string color = (rand() % 2 == 0) ? "Red" : "Blue";
    std::string size_label = (rand() % 2 == 0) ? "Small" : "Large";

    double size = (size_label == "Small") ? small_size_ : large_size_;
    double mass = (size_label == "Small") ? small_mass_ : large_mass_;

    double x = x_dist_(gen_);
    double y = y_dist_(gen_);
    double z = 0.90 + size / 2.0;
    double yaw = yaw_dist_(gen_);

    // Construct box name
    int id = color_counter_[color]++;
    std::ostringstream name;
    name << color << "_" << id;

    // Build SDF model string
    std::ostringstream sdf_str;
    sdf_str << "<sdf version='1.6'>"
            << "<model name='" << name.str() << "'>"
            << "<static>false</static>"
            << "<pose>" << x << " " << y << " " << z << " 0 0 " << yaw << "</pose>"
            << "<link name='link'>"

            // Inertial block
            << "  <inertial>"
            << "    <mass>" << mass << "</mass>"
            << "    <inertia>"
            << "      <ixx>0.001</ixx><iyy>0.001</iyy><izz>0.001</izz>"
            << "      <ixy>0.0</ixy><ixz>0.0</ixz><iyz>0.0</iyz>"
            << "    </inertia>"
            << "  </inertial>"

            // Collision
            << "  <collision name='collision'>"
            << "    <geometry><box><size>" << size << " " << size << " " << size << "</size></box></geometry>"
            << "    <surface>"
            << "      <friction>"
            << "        <ode><mu>2.0</mu><mu2>2.0</mu2></ode>"
            << "      </friction>"
            << "    </surface>"
            << "  </collision>"

            // Visual
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

    sdf::SDF model_sdf;
    model_sdf.SetFromString(sdf_str.str());
    world_->InsertModelSDF(model_sdf);

    res->success = true;
    res->message = "Spawned box: " + name.str();
    RCLCPP_INFO(ros_node_->get_logger(), " Spawned %s (%s %s)", name.str().c_str(), color.c_str(), size_label.c_str());
  }
};

GZ_REGISTER_WORLD_PLUGIN(BoxSpawnerPlugin)

}  // namespace gazebo
