# robot_utils.py
# Utility functions for multi-robot simulation setup.
# Adapted from arshadlab/tb3_multi_robot (Apache License 2.0)

import os
import yaml


def load_sdf_with_namespace(model_path, namespace):
    """Patch SDF file to inject robot namespace into all relevant topic tags."""
    with open(model_path, 'r') as f:
        sdf_text = f.read()

    topic_map = {
        '<tf_topic>/tf</tf_topic>':                           f'<tf_topic>/{namespace}/tf</tf_topic>',
        '<topic>cmd_vel</topic>':                             f'<topic>/{namespace}/cmd_vel</topic>',
        '<odom_topic>odom</odom_topic>':                      f'<odom_topic>/{namespace}/odom</odom_topic>',
        '<frame_id>odom</frame_id>':                          f'<frame_id>{namespace}/odom</frame_id>',
        '<child_frame_id>base_footprint</child_frame_id>':    f'<child_frame_id>{namespace}/base_footprint</child_frame_id>',
        '<topic>joint_states</topic>':                        f'<topic>/{namespace}/joint_states</topic>',
        '<topic>imu</topic>':                                 f'<topic>/{namespace}/imu</topic>',
        '<topic>scan</topic>':                                f'<topic>/{namespace}/scan</topic>',
        '<gz_frame_id>base_scan</gz_frame_id>':               f'<gz_frame_id>{namespace}/base_scan</gz_frame_id>',
    }

    for original, replacement in topic_map.items():
        sdf_text = sdf_text.replace(original, replacement)

    return sdf_text


def create_namespaced_bridge_yaml(base_yaml_path, namespace, include_clock=True):
    """Create a temporary namespaced bridge YAML for ros_gz_bridge.
    
    Args:
        base_yaml_path: Path to the base bridge YAML file.
        namespace: Robot namespace (e.g. 'robot1').
        include_clock: If False, the /clock topic is excluded from the bridge.
                       Set to False for all robots except the first to avoid
                       duplicate clock publishers causing instability.
    """
    with open(base_yaml_path, 'r') as f:
        bridges = yaml.safe_load(f)

    namespace_with_slash = namespace + '/' if not namespace.endswith('/') else namespace
    namespaced_bridges = []

    for bridge in bridges:
        if bridge['ros_topic_name'] == 'clock':
            if not include_clock:
                continue  # skip clock for robot2+, only robot1 bridges it
        else:
            bridge['ros_topic_name'] = f"{namespace_with_slash}{bridge['ros_topic_name']}"
            bridge['gz_topic_name'] = f"{namespace_with_slash}{bridge['gz_topic_name']}"
        namespaced_bridges.append(bridge)

    output_path = f"/tmp/{namespace.strip('/')}_bridge.yaml"
    with open(output_path, 'w') as f:
        yaml.dump(namespaced_bridges, f)

    return output_path