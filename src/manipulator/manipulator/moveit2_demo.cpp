#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <linkattacher_msgs/srv/attach_link.hpp>
#include <linkattacher_msgs/srv/detach_link.hpp>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.hpp>
#endif

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit2_demo");
namespace mtc = moveit::task_constructor;

class MTCTaskNode 
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();
  void setupPlanningScene();
  void attachObjectInGazebo(const std::string& model1, const std::string& link1,
                          const std::string& model2, const std::string& link2);
  void detachObjectInGazebo(const std::string& model1, const std::string& link1,
                          const std::string& model2, const std::string& link2);
  void pickplace();
private:
  mtc::Task createPick(const std::string& object_id);
  mtc::Task pick;
  mtc::Task createPlace(const std::string& object_id);
  mtc::Task place;
  mtc::Task createRetreat();
  mtc::Task retreat;
  rclcpp::Node::SharedPtr node_;
};

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("move2_demo", options) }
{}

void MTCTaskNode::setupPlanningScene()
{
  moveit::planning_interface::PlanningSceneInterface psi;
  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;

  std::vector<std::string> colors = {"red", "blue", "green"};
  std::vector<std::pair<double, double>> positions = {{0.0, 0.6}, {0.4, 0.4}, {0.6, 0.0}};

  for (size_t i = 0; i < colors.size(); ++i) {
    moveit_msgs::msg::CollisionObject box;
    box.id = "square_object_" + colors[i];
    box.header.frame_id = "world";
    box.primitives.resize(1);
    box.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    box.primitives[0].dimensions = {0.025, 0.025, 0.1};

    geometry_msgs::msg::Pose pose;
    pose.position.x = positions[i].first;
    pose.position.y = positions[i].second;
    pose.position.z = 0.05;
    pose.orientation.w = 1.0;
    box.pose = pose;

    collision_objects.push_back(box);
  }

  psi.applyCollisionObjects(collision_objects);
}

void MTCTaskNode::attachObjectInGazebo(const std::string& model1, const std::string& link1,
                                       const std::string& model2, const std::string& link2)
{
  auto client = node_->create_client<linkattacher_msgs::srv::AttachLink>("/ATTACHLINK");

  // Ensure service is available before calling it
  if (!client->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(LOGGER, "ATTACHLINK service not available! Gazebo might not be running.");
    return;
  }

  auto request = std::make_shared<linkattacher_msgs::srv::AttachLink::Request>();
  request->model1_name = model1;
  request->link1_name = link1;
  request->model2_name = model2;
  request->link2_name = link2;

  RCLCPP_INFO(LOGGER, "Attempting to attach in Gazebo: model1='%s' link1='%s' model2='%s' link2='%s'",
             model1.c_str(), link1.c_str(), model2.c_str(), link2.c_str());

  auto result_future = client->async_send_request(request);
  
  // Wait for response while keeping ROS event loop running
  while (result_future.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
    rclcpp::spin_some(node_);
  }

  auto response = result_future.get();
  if (response->success) {
      RCLCPP_INFO(LOGGER, "Successfully attached link in Gazebo!");
  } else {
      RCLCPP_ERROR(LOGGER, "Gazebo ATTACHLINK service returned failure.");
  }

  return;
}

void MTCTaskNode::detachObjectInGazebo(const std::string& model1, const std::string& link1,
                                       const std::string& model2, const std::string& link2)
{
  auto client = node_->create_client<linkattacher_msgs::srv::DetachLink>("/DETACHLINK");

  // Ensure service is available before calling it
  if (!client->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(LOGGER, "DETACHLINK service not available! Gazebo might not be running.");
    return;
  }

  auto request = std::make_shared<linkattacher_msgs::srv::DetachLink::Request>();
  request->model1_name = model1;
  request->link1_name = link1;
  request->model2_name = model2;
  request->link2_name = link2;

  RCLCPP_INFO(LOGGER, "Attempting to detach in Gazebo: model1='%s' link1='%s' model2='%s' link2='%s'",
             model1.c_str(), link1.c_str(), model2.c_str(), link2.c_str());

  auto result_future = client->async_send_request(request);
  
  // Wait for response while keeping ROS event loop running
  while (result_future.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
    rclcpp::spin_some(node_);
  }

  auto response = result_future.get();
  if (response->success) {
      RCLCPP_INFO(LOGGER, "Successfully detached link in Gazebo!");
  } else {
      RCLCPP_ERROR(LOGGER, "Gazebo DETACHLINK service returned failure.");
  }

  return;
}

mtc::Task MTCTaskNode::createPick(const std::string& object_id)
{
  mtc::Task task;
  task.stages()->setName("Pick task");
  task.loadRobotModel(node_);
  const auto& arm_group_name = "manipulator_arm";
  const auto& hand_group_name = "hand";
  const auto& hand_frame = "manipulator_hand";
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);

  // ✅ Verify that the selected object exists
  moveit::planning_interface::PlanningSceneInterface psi;
  auto objects = psi.getObjects({object_id});

  if (objects.find(object_id) == objects.end()) {
    RCLCPP_ERROR(LOGGER, "Object '%s' not found in the planning scene!", object_id.c_str());
    return task;
  }

  RCLCPP_INFO(LOGGER, "Objects in scene: %zu", objects.size());

  mtc::Stage* current_state_ptr = nullptr; 
  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));
  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  sampling_planner->setPlannerId("RRTConnectkConfigDefault");
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(0.2);
  cartesian_planner->setMaxAccelerationScalingFactor(0.2);
  cartesian_planner->setStepSize(0.001);
  auto stage_open_hand = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  stage_open_hand->setGroup(hand_group_name);
  stage_open_hand->setGoal("open");
  task.add(std::move(stage_open_hand));

  auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>("move to pick",
    mtc::stages::Connect::GroupPlannerVector{
      std::make_pair(arm_group_name, sampling_planner)
    });
  stage_move_to_pick->setTimeout(10.0);
  stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(stage_move_to_pick));

  mtc::Stage* attach_object_stage = nullptr; 

  {
    auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
    task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
    grasp->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });
    
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
      stage->properties().set("marker_ns", "approach_object");
      stage->properties().set("link", hand_frame);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.15, 1.0);

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = hand_frame;
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

 
    {
      auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "grasp_pose");
      stage->setPreGraspPose("open");
      stage->setObject(object_id);
      stage->setAngleDelta(M_PI / 24);
      stage->setMonitoredStage(current_state_ptr); 

      Eigen::Isometry3d grasp_frame_transform;
      Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
      grasp_frame_transform.linear() = q.matrix();
      grasp_frame_transform.translation().z() = 0.1;

      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(16);
      wrapper->setMinSolutionDistance(0.5);
      wrapper->setIKFrame(grasp_frame_transform,hand_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      grasp->insert(std::move(wrapper));
    }

    {
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
      stage->allowCollisions(object_id,
                            task.getRobotModel()
                                ->getJointModelGroup(hand_group_name)
                                ->getLinkModelNamesWithCollisionGeometry(),
                            true);
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
      stage->attachObject(object_id, hand_frame);
      grasp->insert(std::move(stage));
    }

    task.add(std::move(grasp));
  }
return task;
}

mtc::Task MTCTaskNode::createPlace(const std::string& object_id)
  {
    mtc::Task task;
    task.stages()->setName("Place task");
    task.loadRobotModel(node_);
    const auto& arm_group_name = "manipulator_arm";
    const auto& hand_group_name = "hand";
    const auto& hand_frame = "manipulator_hand";
    task.setProperty("group", arm_group_name);
    task.setProperty("eef", hand_group_name);
    task.setProperty("ik_frame", hand_frame);

    mtc::Stage* current_state_ptr = nullptr; 
    auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
    current_state_ptr = stage_state_current.get();
    task.add(std::move(stage_state_current));
    auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
    sampling_planner->setPlannerId("RRTConnectkConfigDefault");
    auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScalingFactor(0.2);
    cartesian_planner->setMaxAccelerationScalingFactor(0.2);
    cartesian_planner->setStepSize(0.001);

    mtc::Stage* attach_object_stage = nullptr; 
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.02,0.3);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "lift_object");

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = 1.5;
      stage->setDirection(vec);
      task.add(std::move(stage));
    }

    {
    auto stage_move_to_place = std::make_unique<mtc::stages::Connect>("move to place",
        mtc::stages::Connect::GroupPlannerVector{
          std::make_pair(arm_group_name, sampling_planner),
          std::make_pair(hand_group_name, sampling_planner)
        });
      stage_move_to_place->setTimeout(10.0);
      stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
      task.add(std::move(stage_move_to_place));
    }

    {
      auto place = std::make_unique<mtc::SerialContainer>("place object");
      task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
      place->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

      {
        auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
        stage->properties().configureInitFrom(mtc::Stage::PARENT);
        stage->properties().set("marker_ns", "place_pose");
        stage->setObject(object_id);

        geometry_msgs::msg::PoseStamped target_pose_msg;
        target_pose_msg.header.frame_id = "world";
        target_pose_msg.pose.position.x = 0.4;
        target_pose_msg.pose.position.y = -0.4;
        target_pose_msg.pose.position.z = 0.05;
        target_pose_msg.pose.orientation.w = 1.0;
        stage->setPose(target_pose_msg);
        stage->setMonitoredStage(current_state_ptr);  

        auto wrapper = std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
        wrapper->setMaxIKSolutions(8);
        wrapper->setMinSolutionDistance(1.0);
        wrapper->setIKFrame(object_id);
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
        stage->allowCollisions(object_id,
                              task.getRobotModel()
                                  ->getJointModelGroup(hand_group_name)
                                  ->getLinkModelNamesWithCollisionGeometry(),
                              false);
        place->insert(std::move(stage));
      }

      {
        auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
        stage->detachObject(object_id, hand_frame);
        place->insert(std::move(stage));
      }
     task.add(std::move(place));
    }
  return task;
  }

mtc::Task MTCTaskNode::createRetreat()
{
    mtc::Task task;
    task.stages()->setName("Retreat task");
    task.loadRobotModel(node_);

    const auto& arm_group_name = "manipulator_arm";
    const auto& hand_group_name = "hand";
    const auto& hand_frame = "manipulator_hand";

    task.setProperty("group", arm_group_name);
    task.setProperty("eef", hand_group_name);
    task.setProperty("ik_frame", hand_frame);

    mtc::Stage* current_state_ptr = nullptr;
    auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
    current_state_ptr = stage_state_current.get();
    task.add(std::move(stage_state_current));

    auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
    sampling_planner->setPlannerId("RRTConnectkConfigDefault");
    auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();

    cartesian_planner->setMaxVelocityScalingFactor(0.5);
    cartesian_planner->setMaxAccelerationScalingFactor(0.5);
    cartesian_planner->setStepSize(0.005);

    {
        auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
        stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
        stage->setMinMaxDistance(0.05, 0.3);
        stage->setIKFrame(hand_frame);
        stage->properties().set("marker_ns", "retreat");

        geometry_msgs::msg::Vector3Stamped vec;
        vec.header.frame_id = "world";
        vec.vector.z = 1.5;  
        stage->setDirection(vec);

        task.add(std::move(stage)); 
    }

    {
        auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
        stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
        stage->setGoal("ready");
        task.add(std::move(stage));
    }

    return task;
}
  
void MTCTaskNode::pickplace()
{
  moveit::planning_interface::PlanningSceneInterface psi;
  auto objects = psi.getObjects(); // Get all objects in the planning scene

  if (objects.empty()) {
    RCLCPP_ERROR(LOGGER, "No objects found in the planning scene!");
    return;
  }

  std::string selected_object_id;
  node_->get_parameter("selected_object_id", selected_object_id);
  std::string bot = "manipulator";
  std::string bot_link = "manipulator_link7";
  std::string object_frame = "square_object_link";

  // Check if the requested object exists
  if (objects.find(selected_object_id) == objects.end()) {
    RCLCPP_ERROR(LOGGER, "Object '%s' not found in the planning scene!", selected_object_id.c_str());
    return;
  }

    RCLCPP_INFO(LOGGER, "Object '%s' selected", selected_object_id.c_str());

  pick = createPick(selected_object_id);

  try
  {
    pick.init();
    RCLCPP_INFO(LOGGER, "Pick initialization successful...");
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  RCLCPP_INFO(LOGGER, "Starting Pick Planning...");

  if (!pick.plan(5))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Pick planning failed");
    return;
  }
  RCLCPP_INFO(LOGGER, "Pick Planning completed. Executing...");
  pick.introspection().publishSolution(*pick.solutions().front());
  auto pick_result = pick.execute(*pick.solutions().front());
  if (pick_result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Pick execution failed");
    return ;
  } else {
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    attachObjectInGazebo(bot, bot_link, selected_object_id, object_frame);
    RCLCPP_INFO(LOGGER, "Pick execution completed successfully!");
  }


  place = createPlace(selected_object_id);

  try
  {
    place.init();
    RCLCPP_INFO(LOGGER, "Place initialization successful...");
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  RCLCPP_INFO(LOGGER, "Starting Place Planning...");

  if (!place.plan(5))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Place planning failed");
    return;
  }
  RCLCPP_INFO(LOGGER, "Place Planning completed. Executing...");
  place.introspection().publishSolution(*place.solutions().front());
  auto place_result = place.execute(*place.solutions().front());
  if (place_result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Place execution failed");
  } else {
    detachObjectInGazebo(bot, bot_link, selected_object_id , object_frame);
    RCLCPP_INFO(LOGGER, "Place execution completed successfully!");
  }

  retreat = createRetreat();

  try
  {
    retreat.init();
    RCLCPP_INFO(LOGGER, "Retreat initialization successful...");
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  RCLCPP_INFO(LOGGER, "Starting Retreat Planning...");

  if (!retreat.plan(5))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Retreat planning failed");
    return;
  }
  RCLCPP_INFO(LOGGER, "Retreat Planning completed. Executing...");
  retreat.introspection().publishSolution(*retreat.solutions().front());
  auto retreat_result = retreat.execute(*retreat.solutions().front());
  if (retreat_result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Retreat execution failed");
  } else {
    RCLCPP_INFO(LOGGER, "Retreat execution completed successfully!");
  }

  return;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
 
  mtc_task_node->setupPlanningScene();
  mtc_task_node->pickplace();

  rclcpp::shutdown();
  return 0;
}
