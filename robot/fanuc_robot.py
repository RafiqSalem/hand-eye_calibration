from utils.config_loader import ConfigLoader
from rmi_library.rmi_lib import RMILibrary as rmi_lib
import utils.utils as utils

import numpy as np
import time
import json
import os

class FanucRobot():
    """Universal Robots."""

    def __init__(self, configuration_filepath="configurations\robot_config.json", start_on_creation=True, image_processor=None, ui=None):
        self.configuration_filepath = configuration_filepath
        self.config = self.load_robot_config(configuration_filepath)
        #creates a class which instance attributes are based on the config dictionary
        for k, v in self.config.items():
            setattr(self, k, v)
        self.joint_acc = self.JOINT_ACC_DEFAULT
        self.joint_vel = self.JOINT_VEL_DEFAULT
        self.tool_acc = self.TOOL_ACC_DEFAULT
        self.tool_vel = self.TOOL_VEL_DEFAULT

        self.imp = image_processor
        self.ui = ui
        print(f"Connecting to robot at {self.robot_ip}:{self.tcp_port}...") 
        self.rmi = rmi_lib(robot_ip=self.robot_ip, robot_port=self.tcp_port)

        self.robot_config = (
            {  # Dictionary that defines rotations when more than 1 solution exists, as well as current UTool and UFrame
                "UToolNumber": None,
                "UFrameNumber": None,
                "Front": 1,
                "Up": 1,
                "Left": 0,
                "Flip": 1,
                "Turn4": 0,
                "Turn5": 0,
                "Turn6": 0,
            }
        )

        self.selected_holes_adjusted_probing_pos = []


        self.tcp_pos_uf_probe = None  # Current position of the probe tool in the current user frame
        self.tcp_pos_uf_camera = None  # Current position of the camera tool in the current user frame

        if start_on_creation:
            self.start()



    def load_robot_config(self, file_path):
        """
        Charge les données de configuration du robot à partir d'un fichier JSON.

        Args:
            file_path (str): Le chemin vers le fichier JSON.

        Returns:
            dict: Un dictionnaire contenant les données de configuration.
        """
        if not os.path.exists(file_path):
            raise FileNotFoundError(f"Le fichier de configuration {file_path} n'a pas été trouvé.")
        
        with open(file_path, 'r') as f:
            config_data = json.load(f)
        return config_data

    # Robot methods
    # -------------------------------------
    def go_home(self):
        """Moves the robot to home pose."""
        self.move_joints(self.home_joints_rad)

    def get_state(self):
        return self.rmi.rmi_get_status()

    def get_current_joints(self):
        _, response = self.rmi.rmi_read_joint_angles()

        if response and response.get("ErrorID") == 0:
            print("****test****", response)

            joints = response.get("JointAngle")

            joint_angles = [
                joints["J1"],
                joints["J2"],
                joints["J3"],
                joints["J4"],
                joints["J5"],
                joints["J6"]
            ]

            print(joint_angles)
            return joint_angles

        return [0]*6

    def move_joints(self, joint_configuration_rad):
        joint_angles = {f"J{i+1}": joint_configuration_rad[i] for i in range(6)}
        self.rmi.rmi_joint_motion_JRep(jointAngles=joint_angles, speed_type="mSec", speed=int(self.joint_vel * 100), term_type="FINE", term_value=0)

        # Block until robot reaches desired joint positions.
        current_joints = self.get_current_joints()
        while not all([np.abs(current_joints[i] - joint_configuration_rad[i]) < self.joint_tolerance for i in range(6)]):            
            print("****test****",current_joints[i])
            current_joints = self.get_current_joints()
            time.sleep(0.01)


    # Extension methods
    # -------------------------------------

    def activate_safe_mode(self):
        self.joint_acc = self.JOINT_ACC_SAFE
        self.joint_vel = self.JOINT_VEL_SAFE
        self.tool_acc = self.TOOL_ACC_SAFE
        self.tool_vel = self.TOOL_VEL_SAFE

    def deactivate_safe_mode(self):
        self.joint_acc = self.JOINT_ACC_DEFAULT
        self.joint_vel = self.JOINT_VEL_DEFAULT
        self.tool_acc = self.TOOL_ACC_DEFAULT
        self.tool_vel = self.TOOL_VEL_DEFAULT

    def start(self):
        self.load_program()
        # power_on and brake_release not directly available in rmi_lib

    def load_program(self):
        self.rmi.rmi_call(self.program)

    def get_robot_mode(self):
        response = self.rmi.rmi_get_status()
        return response.get("TPMode", "unknown") if response else "unknown"

    def play_program(self, n_trials=1):
        is_running = self.check_program_running()
        if not is_running:
            self.rmi.rmi_continue()
            mode = self.get_robot_mode()
            if 'AUTO' in mode.upper():
                is_running = self.check_program_running()
                while not is_running:
                    is_running = self.check_program_running()
                    time.sleep(0.01)
            elif n_trials > 0:
                n_trials -=1
                self.play_program(n_trials=n_trials)

    def stop_program(self, n_trials=1):
        self.rmi.rmi_pause()
        mode = self.get_robot_mode()
        if 'PAUSED' in mode.upper():
            is_running = self.check_program_running()
            while is_running:
                is_running = self.check_program_running()
                time.sleep(0.01)
        elif n_trials > 0:
            n_trials -=1
            self.stop_program(n_trials=n_trials)

    def check_program_running(self):
        status = self.rmi.rmi_get_status()
        return status.get("TPMode") == "AUTO" if status else False



    def set_digital_out_signal(self, number, value):
        self.rmi.rmi_write_d_out(number, "ON" if value else "OFF")

    def get_cartesian_pose(self):
        response = self.rmi.rmi_read_cartesian_position()
        if response and response.get("ErrorID") == 0:
            pos = response.get("Position", {})
            return [pos.get("X", 0), pos.get("Y", 0), pos.get("Z", 0), pos.get("W", 0), pos.get("P", 0), pos.get("R", 0)]
        return [0]*6

    def move_to_pose(self, position, orientation):
        config = self.robot_config
        pos_dict = {"X": position[0], "Y": position[1], "Z": position[2], "W": orientation[0], "P": orientation[1], "R": orientation[2]}
        self.rmi.rmi_linear_motion(config=config, position=pos_dict, speed_type="v", speed=self.tool_vel, term_type="FINE", term_value=0)
        #Block until robot reaches desired pose
        current_pose = self.get_cartesian_pose()
        while not all([np.abs(current_pose[i] - position[i]) < self.pose_tolerance[i] for i in range(3)]):
            current_pose = self.get_cartesian_pose()
            time.sleep(0.01)
            print("****************")

    def get_tool_data(self):
        # Not available in rmi_lib
        return 0

    def move_wrt_tool(self, position):
        current_pose = self.get_cartesian_pose()
        current_pose = np.array(current_pose)
        current_position = current_pose[0:3]
        orientation = current_pose[3:6]
        # Transform from position to base_world_position
        position.shape = (3,1)
        current_pose.shape = (1,6)
        T_eb = utils.V2T(current_pose)
        base_world_position = np.dot(T_eb[0:3,0:3], position[0:3,0]) + current_position
        self.move_to_pose(base_world_position[0:3], orientation)

    def orientate_wrt_tool(self, orientation):
        current_pose = self.get_cartesian_pose()
        config = self.robot_config
        pos_dict = {"X": current_pose[0], "Y": current_pose[1], "Z": current_pose[2], "W": orientation[0], "P": orientation[1], "R": orientation[2]}
        self.rmi.rmi_linear_motion(config=config, position=pos_dict, speed_type="v", speed=self.tool_vel, term_type="FINE", term_value=0)

    def get_digital_inputs(self):
        # rmi_read_d_in requires port_number, not available for all
        return []

    def move_with_forces(self, joint_selector, joint_forces, n_seconds=0.7, max_dist_from_orig_m=0.1, max_force_N=90, max_z_velocity=0.05):
        # Force mode not available in rmi_lib
        pass
