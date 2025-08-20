import yaml, os
from ament_index_python.packages import get_package_share_directory

def extract_configuration():
    config_file = os.path.join(
        get_package_share_directory('lidar_zed'),
        'config',
        'general_configuration'
    )

    with open(config_file, 'r') as file:
        config = yaml.safe_load(file)
    
    return config