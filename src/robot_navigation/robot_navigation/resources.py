"""Default hardware settings installed with robot_navigation, never a sibling checkout."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory


def navigation_config():
    return Path(get_package_share_directory('robot_navigation'))/'config/navigation_robot.yaml'
