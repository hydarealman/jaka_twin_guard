from setuptools import setup

package_name = "jaka_dual_arm"

setup(
    name=package_name,
    version="0.1.0",
    packages=[
        package_name,
        package_name + ".planner",
        package_name + ".skills",
        package_name + ".scene",
        package_name + ".behavior",
        package_name + ".behavior.bt_nodes",
    ],
    package_dir={"": "."},
    data_files=[
        ("share/" + package_name + "/config", [
            "config/planner_params.yaml",
            "config/skill_params.yaml",
            "config/scene_params.yaml",
            "config/robot_params.yaml",
            "config/behavior_params.yaml",
        ]),
        ("share/" + package_name + "/launch", [
            "launch/sim_rviz.launch.py",
        ]),
        ("share/" + package_name + "/behavior/trees", [
            "behavior/trees/carry_task.xml",
        ]),
    ],
    entry_points={
        "console_scripts": [
            "carry_task_runner = jaka_dual_arm.__main__:main",
        ],
    },
    install_requires=["setuptools"],
    zip_safe=False,
)
