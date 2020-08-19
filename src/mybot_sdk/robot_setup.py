import os
import yaml

def get_robot_cfg():
    script_path = str(os.path.dirname(os.path.realpath(__file__)))
    config_path = script_path + "/mybot_config.yaml"
    with open(config_path, "r") as ymlfile:
        cfg = yaml.load(ymlfile)
    return cfg
