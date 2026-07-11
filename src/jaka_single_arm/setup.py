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
        ]),
        ("share/" + package_name + "/launch", [
            "launch/sim_rviz.launch.py",
            "launch/sim_gazebo.launch.py",
            "launch/real_hardware.launch.py",
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
    ],
    entry_points={
        "console_scripts": [
            "pick_place_runner = jaka_single_arm.__main__:main",
            "fruit_detector_node = jaka_single_arm.perception.fruit_detector_node:main",
        ],
    },
    install_requires=["setuptools"],
    zip_safe=False,
)
