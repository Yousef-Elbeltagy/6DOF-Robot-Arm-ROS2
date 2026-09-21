from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            "robot_arm",
            package_name="robot_arm_moveit_config",
        )
        .planning_pipelines(
            default_planning_pipeline="ompl",
            pipelines=[
                "ompl",
                "pilz_industrial_motion_planner",
            ],
        )
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    moveit_config.move_group_capabilities["capabilities"] = (
        "pilz_industrial_motion_planner/MoveGroupSequenceAction"
    )

    moveit_config.trajectory_execution["trajectory_execution"] = {
        "allowed_execution_duration_scaling": 2.0,
        "allowed_goal_duration_margin": 2.0,
    }
    moveit_config.robot_description["use_sim_time"] = True
    return generate_move_group_launch(moveit_config)
