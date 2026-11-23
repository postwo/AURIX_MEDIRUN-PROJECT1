from setuptools import find_packages, setup

package_name = "medirun_server"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/server_bringup.launch.py", "launch/server_monitor.launch.py"]),
        (f"share/{package_name}/config", ["config/server.yaml"]),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="aurix",
    maintainer_email="team@example.com",
    description="MediRun central server (headless ROS 2 node).",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "central_server = medirun_server.central_server:main",
        ]
    },
)
