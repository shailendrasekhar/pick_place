#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>

class PlanningSceneNode : public rclcpp::Node
{
public:
    PlanningSceneNode() : Node("planning_scene_node")
    {
        RCLCPP_INFO(get_logger(), "Setting up Planning Scene...");
        setupPlanningScene();
    }

private:
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

    void setupPlanningScene()
    {
        std::vector<moveit_msgs::msg::CollisionObject> collision_objects;

        // Define Red Box
        moveit_msgs::msg::CollisionObject red_box;
        red_box.id = "red_box";
        red_box.header.frame_id = "world";
        red_box.primitives.resize(1);
        red_box.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
        red_box.primitives[0].dimensions = {0.025, 0.025, 0.1}; // Box dimensions

        geometry_msgs::msg::Pose red_pose;
        red_pose.position.x = 0.0;
        red_pose.position.y = 0.6;
        red_pose.position.z = 0.05; // Half of height to sit on the ground
        red_pose.orientation.w = 1.0;
        red_box.pose = red_pose;

        collision_objects.push_back(red_box);

        // Define Blue Box
        moveit_msgs::msg::CollisionObject blue_box;
        blue_box.id = "blue_box";
        blue_box.header.frame_id = "world";
        blue_box.primitives.resize(1);
        blue_box.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
        blue_box.primitives[0].dimensions = {0.025, 0.025, 0.1};

        geometry_msgs::msg::Pose blue_pose;
        blue_pose.position.x = 0.4;
        blue_pose.position.y = 0.4;
        blue_pose.position.z = 0.05;
        blue_pose.orientation.w = 1.0;
        blue_box.pose = blue_pose;

        collision_objects.push_back(blue_box);

        // Define Green Box
        moveit_msgs::msg::CollisionObject green_box;
        green_box.id = "green_box";
        green_box.header.frame_id = "world";
        green_box.primitives.resize(1);
        green_box.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
        green_box.primitives[0].dimensions = {0.025, 0.025, 0.1};

        geometry_msgs::msg::Pose green_pose;
        green_pose.position.x = 0.8;
        green_pose.position.y = 0.2;
        green_pose.position.z = 0.05;
        green_pose.orientation.w = 1.0;
        green_box.pose = green_pose;

        collision_objects.push_back(green_box);

        // Apply the objects to the planning scene
        planning_scene_interface_.applyCollisionObjects(collision_objects);

        RCLCPP_INFO(get_logger(), "Planning Scene Setup Complete!");
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PlanningSceneNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
