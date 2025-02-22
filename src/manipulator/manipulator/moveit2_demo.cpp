#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

// namespace alias for convinience
static const rclcpp::Logger LOGGER = rclcpp::get_logger("manipulator");
namespace mtc = moveit::task_constructor;

// class for MoveIT Task Constructor Funcationality
class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void doTask();

  void setupPlanningScene();

private:
  // Compose an MTC task from a series of stages.
  mtc::Task createTask();
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
};

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

// Initialize the node with specified Options
MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("manipulator_moveit2_demo", options) }
{
}


// Class to set up Planning Scene

void MTCTaskNode::setupPlanningScene()
{

  geometry_msgs::msg::Pose pose;
  moveit_msgs::msg::CollisionObject object;
  object.id = "object";
  object.header.frame_id = "world";
  object.primitives.resize(1);

  // CUBE
  // object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX; 
  // object.primitives[0].dimensions = { 0.04, 0.04, 0.04 }; 

  // CUBOID
  // object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX; 
  // object.primitives[0].dimensions = { 0.06, 0.03, 0.1 }; 
 

  // CYLINDER
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER; 
  object.primitives[0].dimensions = { 0.1,0.02 }; 

  // SPHERE
  // object.primitives[0].type = shape_msgs::msg::SolidPrimitive::SPHERE; 
  // object.primitives[0].dimensions = {0.02}; 

  // CONE
  // object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CONE; 
  // object.primitives[0].dimensions = { 0.1,0.02 }; 

  pose.position.x = 0.4;
  pose.position.y = 0.0;
  pose.position.z = 0.0;

  pose.orientation.w = 0.707;
  pose.orientation.x = 0.707;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;

  object.pose = pose;

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object);
}


// Class to create a task
void MTCTaskNode::doTask()
{
  task_ = createTask();

  try
  {
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  if (!task_.plan(10))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return;
  }
  task_.introspection().publishSolution(*task_.solutions().front());

  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
    return;
  }

  return;
}

mtc::Task MTCTaskNode::createTask()
{
  mtc::Task task;
  task.stages()->setName("demo task");
  task.loadRobotModel(node_);

  const auto& arm_group_name = "manipulator_arm";
  const auto& hand_group_name = "hand";
  const auto& hand_frame = "manipulator_hand";

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);

// Disable warnings for this line, as it's a variable that's set but not used in this example
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
  mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
#pragma GCC diagnostic pop

  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(0.5);
  cartesian_planner->setMaxAccelerationScalingFactor(0.5);
  cartesian_planner->setStepSize(0.01);
  cartesian_planner->setJumpThreshold(0.0);

  auto stage_open_hand =
      std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  stage_open_hand->setGroup(hand_group_name);
  stage_open_hand->setGoal("open");
  task.add(std::move(stage_open_hand));

//////////////////////////////////////////////////////////////////////////////////////////////////

  auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
    "move to pick",
    mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
  stage_move_to_pick->setTimeout(5.0);
  stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(stage_move_to_pick));

  mtc::Stage* attach_object_stage = 
    nullptr;  // Forward attach_object_stage to place pose generator

  {
  auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
  task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
  grasp->properties().configureInitFrom(mtc::Stage::PARENT,
                                        { "eef", "group", "ik_frame" });


    {
    auto stage =
        std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
    stage->properties().set("marker_ns", "approach_object");
    stage->properties().set("link", hand_frame);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setMinMaxDistance(0.03, 0.10);

    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = hand_frame;
    vec.vector.z = 1.0;
    stage->setDirection(vec);
    grasp->insert(std::move(stage));
    }

    {

    Eigen::Isometry3d grasp_frame_transform;
    Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()) *
                          Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()) *
                          Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
    grasp_frame_transform.linear() = q.matrix();
    grasp_frame_transform.translation().z() = 0.15;  // Increase standoff distance
    grasp_frame_transform.translation().x() = 0.02;  // Add slight offset in x direction

    // Sample grasp pose
    auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    stage->properties().set("marker_ns", "grasp_pose");
    stage->setPreGraspPose("open");
    stage->setObject("object");
    stage->setAngleDelta(M_PI / 6);  // Adjust angle delta
    stage->setMonitoredStage(current_state_ptr);

   

    // Compute IK
    auto wrapper =
        std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
    wrapper->setMaxIKSolutions(8);
    wrapper->setMinSolutionDistance(1.0);
    wrapper->setIKFrame(grasp_frame_transform, hand_frame);
    wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
    wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
    grasp->insert(std::move(wrapper));
    }

    {
    auto stage =
        std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
    
    // Create a vector of link names
    std::vector<std::string> allowed_links = {"manipulator_rightfinger", "manipulator_leftfinger", "manipulator_hand"};
    
    // Allow collisions for each link individually
    for (const auto& link : allowed_links) {
        stage->allowCollisions("object", link, true);
    }
    
    // Also allow collisions with the hand group links
    stage->allowCollisions("object",
                          task.getRobotModel()
                              ->getJointModelGroup(hand_group_name)
                              ->getLinkModelNamesWithCollisionGeometry(),
                          true);
    grasp->insert(std::move(stage));
    }

    {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("maintain hand object collision");
    std::vector<std::string> allowed_links = {"manipulator_rightfinger", "manipulator_leftfinger", "manipulator_hand"};
    for (const auto& link : allowed_links) {
        stage->allowCollisions("object", link, true);
    }
    grasp->insert(std::move(stage));
    }

    {
    auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
    stage->setGroup(hand_group_name);
    stage->setGoal("close");
    grasp->insert(std::move(stage));
    }

    {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
    stage->attachObject("object", hand_frame);
    attach_object_stage = stage.get();
    grasp->insert(std::move(stage));
    }
    
    {
    auto stage =
        std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setMinMaxDistance(0.03, 0.08);  // Reduce lift distance
    stage->setIKFrame(hand_frame);
    stage->properties().set("marker_ns", "lift_object");

    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = "world";
    vec.vector.z = 1.0;
    stage->setDirection(vec);
    grasp->insert(std::move(stage));
    }

    task.add(std::move(grasp));
  }

////////////////////////////////////////////////////////////////////////////////////////////////////

  {
  auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
      "move to place",
      mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner },
                                                { hand_group_name, interpolation_planner } });
  stage_move_to_place->setTimeout(5.0);
  stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(stage_move_to_place));
  }

  {
    auto place = std::make_unique<mtc::SerialContainer>("place object");
    task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
    place->properties().configureInitFrom(mtc::Stage::PARENT,
                                          { "eef", "group", "ik_frame" });

    {
      auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "place_pose");
      stage->setObject("object");

      geometry_msgs::msg::PoseStamped target_pose_msg;
      target_pose_msg.header.frame_id = "object";
      target_pose_msg.pose.position.x = 0.5;
      target_pose_msg.pose.position.y = 0.0;
      target_pose_msg.pose.position.z = 0.0;
      target_pose_msg.pose.orientation.w = 0.707;
      target_pose_msg.pose.orientation.x = 0.707;
      target_pose_msg.pose.orientation.y = 0.0;
      target_pose_msg.pose.orientation.z = 0.0;
      stage->setPose(target_pose_msg);
      stage->setMonitoredStage(attach_object_stage);  // Hook into attach_object_stage

      auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(2);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame("object");
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      place->insert(std::move(wrapper));
    }

    {
    auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
    stage->setGroup(hand_group_name);
    stage->setGoal("open");
    place->insert(std::move(stage));
    }

    {
    auto stage =
        std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
    stage->allowCollisions("object",
                          task.getRobotModel()
                              ->getJointModelGroup(hand_group_name)
                              ->getLinkModelNamesWithCollisionGeometry(),
                          false);
    place->insert(std::move(stage));
    }

    {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
    stage->detachObject("object", hand_frame);
    place->insert(std::move(stage));
    }

    {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setMinMaxDistance(0.05, 0.15);
    stage->setIKFrame(hand_frame);
    stage->properties().set("marker_ns", "retreat");

    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = "world";
    vec.vector.x = -0.5;
    vec.vector.y = 0.0;
    vec.vector.z = 0.0;   
    
    stage->setDirection(vec);
    place->insert(std::move(stage));
    }
    task.add(std::move(place));
  }

  {
  auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
  stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage->setGoal("ready");
  task.add(std::move(stage));
  }
  return task;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface());
  });

  mtc_task_node->setupPlanningScene();
  mtc_task_node->doTask();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}