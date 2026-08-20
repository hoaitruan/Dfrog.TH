from setuptools import find_packages, setup
import os
from glob import glob

package_name = "px4_vslam_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*.launch.py")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Drone Pilot",
    maintainer_email="user@example.com",
    description="cuVSLAM ground-truth TF + PX4 EKF2 external vision bridge",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ground_truth_tf = px4_vslam_bridge.ground_truth_tf:main",
            "vslam_compare = px4_vslam_bridge.vslam_compare:main",
            "visual_odometry_bridge = px4_vslam_bridge.visual_odometry_bridge:main",
            "reactive_esdf_avoidance = px4_vslam_bridge.reactive_esdf_avoidance:main",
            "fast_planner_bridge = px4_vslam_bridge.fast_planner_bridge:main",
            "fast_planner_trigger = px4_vslam_bridge.fast_planner_trigger:main",
        ],
    },
)
