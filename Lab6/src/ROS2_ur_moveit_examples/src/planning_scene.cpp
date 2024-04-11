
// example no 4
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit/kinematic_constraints/utils.h>

int main(int argc, char *argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "hello_moveit",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // We spin up a SingleThreadedExecutor for the current state monitor to get
  // information about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]()
                             { executor.spin(); });

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");

  RCLCPP_ERROR(logger, "test");
  RCLCPP_ERROR(logger, "getDefaultPlanningPipelineId: %s", move_group_interface.getDefaultPlanningPipelineId().c_str());

  // Construct and initialize MoveItVisualTools
  auto moveit_visual_tools =
      moveit_visual_tools::MoveItVisualTools{node, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC,
                                             move_group_interface.getRobotModel()};
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();

  // Create a closure for updating the text in rviz
  // auto const draw_title = [&moveit_visual_tools](auto text) {
  //   auto const text_pose = [] {
  //     auto msg = Eigen::Isometry3d::Identity();
  //     msg.translation().z() = 1.0;
  //     return msg;
  //   }();
  //   moveit_visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  // };
  // auto const prompt = [&moveit_visual_tools](auto text) { moveit_visual_tools.prompt(text); };
  auto const draw_trajectory_tool_path =
      [&moveit_visual_tools, jmg = move_group_interface.getRobotModel()->getJointModelGroup("ur_manipulator")](
          auto const trajectory)
  { moveit_visual_tools.publishTrajectoryLine(trajectory, jmg); };

  // Set a target Pose
  auto const target_pose = []
  {
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = -1.0;
    msg.position.x = -0.18;
    msg.position.y = 0.3;
    // change to values when executed:
    msg.position.x = -0.28;
    msg.position.y = -0.1;
    msg.position.z = 0.1;
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose);

  // Create 1st collision object for the robot to avoid
  auto const collision_object = [frame_id =
                                     move_group_interface.getPlanningFrame()]
  {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = "box1";
    shape_msgs::msg::SolidPrimitive primitive;

    // Define the size of the box in meters
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.5;
    primitive.dimensions[primitive.BOX_Y] = 0.1;
    primitive.dimensions[primitive.BOX_Z] = 0.5;

    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = -0.5;
    box_pose.position.y = 0.2;
    box_pose.position.z = 0.25;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    return collision_object;
  }();

  // Create 2nd collision object for the robot to avoid
  auto const collision_object1 = [frame_id =
                                     move_group_interface.getPlanningFrame()]
  {
    moveit_msgs::msg::CollisionObject collision_object1;
    collision_object1.header.frame_id = frame_id;
    collision_object1.id = "box2";
    shape_msgs::msg::SolidPrimitive primitive;

    // Define the size of the box in meters
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.3;
    primitive.dimensions[primitive.BOX_Y] = 0.3;
    primitive.dimensions[primitive.BOX_Z] = 0.5;

    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 0.5;
    box_pose.position.x = -0.15;
    box_pose.position.y = -0.35;
    box_pose.position.z = 0.25;

    collision_object1.primitives.push_back(primitive);
    collision_object1.primitive_poses.push_back(box_pose);
    collision_object1.operation = collision_object1.ADD;

    return collision_object1;
  }();

  // Create 3rd collision object for the robot to avoid
  auto const collision_object2 = [frame_id =
                                     move_group_interface.getPlanningFrame()]
  {
    moveit_msgs::msg::CollisionObject collision_object2;
    collision_object2.header.frame_id = frame_id;
    collision_object2.id = "box3";
    shape_msgs::msg::SolidPrimitive primitive;

    // Define the size of the box in meters
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.25;
    primitive.dimensions[primitive.BOX_Y] = 0.25;
    primitive.dimensions[primitive.BOX_Z] = 0.25;

    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 0.0;
    box_pose.position.x = 0.50;
    box_pose.position.y = 0.0;
    box_pose.position.z = 0.0;

    collision_object2.primitives.push_back(primitive);
    collision_object2.primitive_poses.push_back(box_pose);
    collision_object2.operation = collision_object2.ADD;

    return collision_object2;
  }();

  // Add the collision object to the scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.applyCollisionObject(collision_object);
  planning_scene_interface.applyCollisionObject(collision_object1);
  planning_scene_interface.applyCollisionObject(collision_object2);

  // Create a plan to that target pose
  // prompt("Press 'next' in the RvizVisualToolsGui window to plan");
  // draw_title("Planning");
  // moveit_visual_tools.trigger();
  RCLCPP_ERROR(logger, "Planning");
  auto const [success, plan] = [&move_group_interface]
  {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if (success)
  {
    draw_trajectory_tool_path(plan.trajectory_);
    // draw_title("Executing");
    RCLCPP_ERROR(logger, "Executing");
    move_group_interface.execute(plan);
  }
  else
  {
    // draw_title("Planning Failed!");
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  spinner.join();
  return 0;
}