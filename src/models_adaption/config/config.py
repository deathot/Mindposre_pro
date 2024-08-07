import yaml


class ModelConfig:
    def __init__(self, yaml_file='config.yaml'):
        self.yaml_data = self.yaml_load(yaml_file)

    @staticmethod
    def yaml_load(yaml_file='data.yaml'):
        # Single-line safe yaml loading
        with open(yaml_file, errors='ignore') as f:
            return yaml.safe_load(f)

    def yaml_dump(self, yaml_file):
        with open(yaml_file, "w+", encoding="utf-8") as f:
            for key, val in self.yaml_data.items():
                yaml.dump({key: val}, f, default_flow_style=False, sort_keys=False)

    def set_para(self, model, key, value):
        self.yaml_data[model][key] = value

    def set_yaml_data(self, yaml_data):
        self.yaml_data = yaml_data

    def get_yaml_data(self):
        return self.yaml_data
