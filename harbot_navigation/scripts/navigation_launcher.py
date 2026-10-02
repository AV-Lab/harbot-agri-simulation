#!/usr/bin/env python3


# ******************************************************************************
# Code Summary:
# - Read the navigation role and selected component name from ROS parameters.
# - Find the component's node.py and config.yaml files.
# - Load config settings into ROS without overwriting existing parameters.
# - Import and create the component class, then keep the ROS node running.
# ******************************************************************************


import os
import sys
import yaml
import rospy
import rospkg
import importlib.util


ROLE_DIRECTORIES = {
    "global_planner": "global_planners",
    "local_planner": "local_planners",
    "speed_planner": "speed_planners",
    "controller": "controllers",
}


class NavigationLauncher:
    def __init__(self):
        self.role = rospy.get_param("~role")
        if self.role not in ROLE_DIRECTORIES:
            raise ValueError("Unknown navigation role '{}'".format(self.role))

        selection_param = "{}{}".format(
            rospy.get_namespace(),
            self.role,
        )
        self.component_name = str(rospy.get_param(selection_param)).strip()

        package_path = rospkg.RosPack().get_path("harbot_navigation")
        sys.path.insert(0, os.path.join(package_path, "scripts"))
        component_path = os.path.join(
            package_path,
            "scripts",
            ROLE_DIRECTORIES[self.role],
            self.component_name,
        )
        node_path = os.path.join(component_path, "node.py")
        config_path = os.path.join(component_path, "config.yaml")

        component_config = self.load_config(config_path)
        self.load_parameters(component_config.get("parameters", {}))
        component_class = self.load_class(
            node_path,
            component_config.get("entry_class"),
        )

        rospy.loginfo(
            "Using %s component: %s",
            self.role,
            self.component_name,
        )
        self.component = component_class()

    def load_config(self, config_path):
        with open(config_path, "r", encoding="utf-8") as config_file:
            config = yaml.safe_load(config_file) or {}

        if (
            not isinstance(config, dict)
            or not config.get("entry_class")
            or not isinstance(config.get("parameters", {}), dict)
        ):
            raise ValueError("Invalid component config: check entry_class and parameters")

        return config

    def load_parameters(self, parameters):
        for parameter_name, value in parameters.items():
            private_name = "~{}".format(parameter_name)
            if not rospy.has_param(private_name):
                rospy.set_param(private_name, value)

    def load_class(self, node_path, class_name):
        if not os.path.isfile(node_path):
            raise ValueError("Component node file not found: {}".format(node_path))

        module_name = "harbot_navigation_{}_{}".format(
            self.role,
            self.component_name,
        )
        # Load the file at node_path using this module name.
        module_spec = importlib.util.spec_from_file_location(module_name, node_path)

        # Raises an error if Python cannot prepare a loader.
        if module_spec is None or module_spec.loader is None:
            raise ValueError("Could not load component: {}".format(node_path))

        # Create the module and load its code.
        module = importlib.util.module_from_spec(module_spec)
        module_spec.loader.exec_module(module)

        # Find the requested class.
        component_class = getattr(module, class_name, None)
        if component_class is None:
            raise ValueError(
                "Component {} does not define class {}".format(
                    node_path,
                    class_name,
                )
            )
        return component_class


def main():
    rospy.init_node("navigation_launcher")
    NavigationLauncher()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
