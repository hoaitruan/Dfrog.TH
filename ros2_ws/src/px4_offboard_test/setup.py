from setuptools import find_packages, setup

package_name = "px4_offboard_test"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Drone Pilot",
    maintainer_email="user@example.com",
    description="Phase 1 gate: minimal Offboard hold using PX4's own sim state estimate",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "minimal_offboard = px4_offboard_test.minimal_offboard:main",
        ],
    },
)
