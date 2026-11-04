"""Publish the pinned YAM model and TF from /yam/joint_states."""

from pathlib import Path
import xml.etree.ElementTree as ET

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    model = Path(__file__).resolve().parent / "vendor/i2rt/i2rt/robot_models/arm/yam/v1/yam.urdf"
    root = ET.parse(model).getroot()
    # The upstream URDF uses relative meshes; RViz needs resolvable file URIs.
    for mesh in root.iter("mesh"):
        filename = mesh.attrib["filename"]
        if "://" not in filename:
            mesh.set("filename", (model.parent / filename).resolve().as_uri())
    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace="yam",
            parameters=[{
                "robot_description": ParameterValue(ET.tostring(root, encoding="unicode"), value_type=str),
                "frame_prefix": "yam/",
            }],
            remappings=[("joint_states", "/yam/joint_states")],
        ),
    ])
