#!/usr/bin/env python3

import rospy
import sys
import os
import yaml
from omni4.generate_c_code import main as generate_omni4_libs
from diff.generate_c_code import main as generate_diff_libs
from tric.generate_c_code import main as generate_tric_libs

if __name__ == '__main__':
    rospy.init_node('generate_acados_libs', anonymous=True)
    
    try:  
        if len(sys.argv) > 1: # Check if a custom YAML file path is provided
            custom_yaml_path = sys.argv[1]

            rospy.loginfo(f"Using parameters from: {custom_yaml_path}")
            rospy.loginfo("Generating Acados solver libraries...")

            with open(custom_yaml_path, 'r') as file:
                params = yaml.safe_load(file)

            script_dir = os.path.dirname(os.path.abspath(__file__))
            if "omni4_params" in params:
                omni4_output_dir = os.path.join(script_dir, 'omni4')
                os.chdir(omni4_output_dir)  # Change to 'omni4' folder
                generate_omni4_libs(params["omni4_params"])  # Generate 'omni4' code
                rospy.loginfo(f"Generated 'omni4' libraries in {omni4_output_dir}.")
            else:
                rospy.logwarn("No parameters found in YAML file to generate the 'omni4' libraries.")
            
            if "diff_params" in params:
                diff_output_dir = os.path.join(script_dir, 'diff')
                os.chdir(diff_output_dir)  # Change to 'diff' folder
                generate_diff_libs(params["diff_params"])  # Generate 'diff' code
                rospy.loginfo(f"Generated 'diff' libraries in {diff_output_dir}.")
            else:
                rospy.logwarn("No parameters found in YAML file to generate the 'diff' libraries.")
            
            if "tric_params" in params:
                tric_output_dir = os.path.join(script_dir, 'tric')
                os.chdir(tric_output_dir)  # Change to 'tric' folder
                generate_tric_libs(params["tric_params"])  # Generate 'tric' code
                rospy.loginfo(f"Generated 'tric' libraries in {tric_output_dir}.")
            else:
                rospy.logwarn("No parameters found in YAML file to generate the 'tric' libraries.")

            rospy.loginfo("Acados solver libraries generated successfully.")
        else:
            rospy.logerr("The path for the YAML file is required to generate the Acados solver libraries!")

    except Exception as e:
        rospy.logerr(f"Failed to generate Acados solver libraries: {e}")