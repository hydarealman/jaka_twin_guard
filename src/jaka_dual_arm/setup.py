from setuptools import setup

package_name = "jaka_dual_arm"

setup(
    name=package_name,
    version="0.3.0",
    packages=[
        package_name,
        package_name + ".planner",
        package_name + ".skills",
        package_name + ".scene",
        package_name + ".behavior",
        package_name + ".behavior.bt_nodes",
        package_name + ".control",
        package_name + ".hardware",
        package_name + ".massage",
    ],
    package_dir={"": "."},
    data_files=[
        ("share/" + package_name + "/config", [
            "config/planner_params.yaml",
            "config/skill_params.yaml",
            "config/scene_params.yaml",
            "config/scene_a_table_pick.yaml",
            "config/scene_b_bin_pick.yaml",
            "config/scene_c_conveyor.yaml",
            "config/robot_params.yaml",
            "config/behavior_params.yaml",
            "config/impedance_params.yaml",
            "config/safety_params.yaml",
            "config/cartesian_impedance_controller.yaml",
            "config/real_hardware_params.yaml",
            "config/massage_body_params.yaml",
            "config/massage_stages.yaml",
        ]),
        ("share/" + package_name + "/launch", [
            "launch/sim_rviz.launch.py",
            "launch/sim_gazebo.launch.py",
            "launch/sim_gazebo_massage.launch.py",
            "launch/real_dual_arm.launch.py",
            "launch/industrial_massage.launch.py",
        ]),
        ("share/" + package_name + "/worlds", [
            "worlds/scene_a_table_pick.world",
            "worlds/scene_b_bin_pick.world",
            "worlds/scene_c_conveyor.world",
            "worlds/massage.world",
        ]),
        ("share/" + package_name + "/behavior/trees", [
            "behavior/trees/carry_task.xml",
            "behavior/trees/massage_task.xml",
        ]),
    ],
    entry_points={
        "console_scripts": [
            "carry_task_runner = jaka_dual_arm.__main__:main",
            "massage_runner = jaka_dual_arm.massage.__main__:main",
        ],
    },
    install_requires=["setuptools"],
    zip_safe=False,
)
