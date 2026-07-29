from setuptools import setup

package_name = "jaka_single_arm"

setup(
    name=package_name,
    version="0.1.0",
    packages=[
        package_name,
        package_name + ".planner",
        package_name + ".skills",
        package_name + ".scene",
        package_name + ".perception",
        package_name + ".behavior",
        package_name + ".behavior.bt_nodes",
        package_name + ".control",
        package_name + ".communication",
    ],
    package_dir={"": "."},
    data_files=[
        ("share/" + package_name + "/config", [
            "config/robot_params.yaml",
            "config/scene_params.yaml",
            "config/perception_params.yaml",
            "config/planner_params.yaml",
            "config/skill_params.yaml",
            "config/behavior_params.yaml",
            "config/safety_params.yaml",
            "config/gripper_params.yaml",
            "config/architecture_a_serial.yaml",
            "config/architecture_b_serial.yaml",
            "config/hand_eye_params.yaml",
        ]),
        ("share/" + package_name + "/launch", [
            "launch/sim_rviz.launch.py",
            "launch/sim_gazebo.launch.py",
            "launch/real_hardware.launch.py",
            "launch/architecture_a_moveit_serial.launch.py",
            "launch/architecture_b_target_serial.launch.py",
            "launch/architecture_a_sim.launch.py",
            "launch/architecture_a_real.launch.py",
            "launch/architecture_b_sim.launch.py",
            "launch/architecture_b_real.launch.py",
        ]),
        ("share/" + package_name + "/worlds", [
            "worlds/pick_place.world",
        ]),
        ("share/" + package_name + "/worlds/materials/scripts", [
            "worlds/materials/scripts/apple.material",
        ]),
        ("share/" + package_name + "/worlds/materials/textures", [
            "worlds/materials/textures/healthy_apple.jpg",
            "worlds/materials/textures/unhealthy_apple.jpg",
        ]),
        ("share/" + package_name + "/models", [
            "models/best.onnx",
            "models/best.pt",
            "models/data.yaml",
        ]),
        ("share/" + package_name + "/behavior/trees", [
            "behavior/trees/pick_place_task.xml",
        ]),
        ("share/" + package_name + "/docs", [
            "docs/CONTROL_ARCHITECTURES.md",
            "docs/SERIAL_CONTROL_PROTOCOL.md",
            "docs/RUN_MODES.md",
            "docs/MODEL_LICENSE_AUDIT.md",
            "docs/CUSTOM_ARM_CAD_MODEL.md",
        ]),
    ],
    entry_points={
        "console_scripts": [
            "pick_place_runner = jaka_single_arm.__main__:main",
            "fruit_detector_node = jaka_single_arm.perception.fruit_detector_node:main",
            "fruit_target_node = jaka_single_arm.perception.fruit_target_node:main",
            "hand_eye_static_tf = jaka_single_arm.perception.hand_eye_static_tf:main",
            "serial_trajectory_controller = jaka_single_arm.communication.serial_trajectory_controller:main",
            "serial_fruit_target_bridge = jaka_single_arm.communication.serial_fruit_target_bridge:main",
            "serial_board_emulator = jaka_single_arm.communication.board_emulator:main",
        ],
    },
    install_requires=["setuptools"],
    zip_safe=False,
)
