import os.path
from typing import Callable
from src.manager.libs.applications.compatibility.exercise_wrapper_ros2 import CompatibilityExerciseWrapperRos2

class Exercise(CompatibilityExerciseWrapperRos2):
    def __init__(self, circuit: str, update_callback: Callable):
        current_path = os.path.dirname(__file__)

        super(Exercise, self).__init__(exercise_command="/workspace/worlds/controller.py 0.0.0.0",
                                       gui_command="/workspace/worlds/controller.py 0.0.0.0",
                                       update_callback=update_callback)
