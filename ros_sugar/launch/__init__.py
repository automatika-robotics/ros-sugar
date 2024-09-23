"""ros_sugar launch tools"""

import launch

LOGGER_NAME = "Launcher"

logger = launch.logging.get_logger(LOGGER_NAME)

__all__ = ["logger"]
