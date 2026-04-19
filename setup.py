from glob import glob

from setuptools import find_packages, setup


package_name = "offboard_px4_course"


setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{package_name}"],
        ),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/worlds", glob("worlds/*.sdf")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="admin",
    maintainer_email="admin@example.com",
    description="PX4 offboard ROS 2 course package.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "offboard = offboard_px4_course.offboard:main",
            "takeofflanding = offboard_px4_course.takeofflanding:main",
            "takeoff_point_hold = offboard_px4_course.takeoff_point_hold:main",
            "mono_aruco_takeoff = offboard_px4_course.mono_aruco_takeoff:main",
        ],
    },
)
