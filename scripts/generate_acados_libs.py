#!/usr/bin/env python3

import sys
import os
import yaml
import logging
from omni4.generate_c_code import main as generate_omni4_libs
from diff.generate_c_code import main as generate_diff_libs
from tric.generate_c_code import main as generate_tric_libs

def setup_logger():
    logging.basicConfig(level=logging.INFO, format='[%(levelname)s] %(message)s')

def main():
    setup_logger()

    if len(sys.argv) > 1:  # Check if a custom YAML file path is provided
        custom_yaml_path = sys.argv[1]

        logging.info(f"Using parameters from: {custom_yaml_path}")
        logging.info("Generating Acados solver libraries...")

        try:
            with open(custom_yaml_path, 'r') as file:
                params = yaml.safe_load(file)

            script_dir = os.path.dirname(os.path.abspath(__file__))

            if "omni4_params" in params:
                omni4_output_dir = os.path.join(script_dir, 'omni4')
                os.chdir(omni4_output_dir)
                generate_omni4_libs(params["omni4_params"])
                logging.info(f"Generated 'omni4' libraries in {omni4_output_dir}.")
            else:
                logging.warning("No parameters found in YAML file to generate the 'omni4' libraries.")

            if "diff_params" in params:
                diff_output_dir = os.path.join(script_dir, 'diff')
                os.chdir(diff_output_dir)
                generate_diff_libs(params["diff_params"])
                logging.info(f"Generated 'diff' libraries in {diff_output_dir}.")
            else:
                logging.warning("No parameters found in YAML file to generate the 'diff' libraries.")

            if "tric_params" in params:
                tric_output_dir = os.path.join(script_dir, 'tric')
                os.chdir(tric_output_dir)
                generate_tric_libs(params["tric_params"])
                logging.info(f"Generated 'tric' libraries in {tric_output_dir}.")
            else:
                logging.warning("No parameters found in YAML file to generate the 'tric' libraries.")

            logging.info("Acados solver libraries generated successfully.")

        except Exception as e:
            logging.error(f"Failed to generate Acados solver libraries: {e}")
    else:
        logging.error("The path for the YAML file is required to generate the Acados solver libraries!")

if __name__ == '__main__':
    main()
