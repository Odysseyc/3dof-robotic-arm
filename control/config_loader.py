import yaml


def load_robot_config():

    with open("config/robot_params.yaml", "r") as file:
        return yaml.safe_load(file)
