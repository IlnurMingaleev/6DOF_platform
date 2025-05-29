# Description: This file is used to test the stewart platform
import pybullet as p
import time
import pybullet_data
import os 
from inv_kinematics import inv_kinematics as ik
import numpy as np

class StewartPlatform:

    def __init__(self, path, joint_indices, actuator_indices, design_variable) -> None:
        self.robotId = None
        self.path = path
        self.joint_indices = joint_indices
        self.actuator_indices = actuator_indices
        self.design_variable = design_variable
        self.prev_target = np.zeros(len(actuator_indices))
        self.current_pose = np.zeros(6)  # [pitch, roll, yaw, sway, surge, heave]
        self.sine_active = False
        self.sine_params = None
        self.sine_start_time = None
        self.sim_env_ready = False
        self.set_env()

    def cls(self):
        os.system('cls')
    def set_env(self):
        if self.sim_env_ready:
            return
        physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
        # physicsClient = p.connect(p.DIRECT)#or p.DIRECT for non-graphical version
        p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
        p.setGravity(0,0,-9.81)
        planeId = p.loadURDF("plane.urdf")
        cubeStartPos = [0,0,0]
        cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
        # Load the Stewart Platform
        self.robotId = p.loadURDF(self.path,cubeStartPos, cubeStartOrientation,
                                flags=p.URDF_USE_INERTIA_FROM_FILE,useFixedBase = 1)
                        
        # Set the camera position and orientation
        camera_target_position = [0, 0, 0]
        camera_distance = 1.5
        camera_yaw = 50
        camera_pitch = -35
        p.resetDebugVisualizerCamera(cameraDistance = camera_distance, 
                                    cameraYaw = camera_yaw,
                                    cameraPitch = camera_pitch,
                                    cameraTargetPosition = camera_target_position, 
                                    physicsClientId = physicsClient)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
        self.n = p.getNumJoints(self.robotId)  # Get the number of joints in the robot
        self.Ind = {}                            # Create a dictionary to store the joint indices
        # Get the joint indices
        for i in range(self.n):
            joint_info = p.getJointInfo(self.robotId, i)
            joint_name = joint_info[1].decode("utf-8")
            self.Ind[joint_info[0]]=joint_name
            # p.changeVisualShape(self.robotId, i, rgbaColor=[1,0,0,1])
        self.sim_env_ready = True
        return

    def set_constraints(self):
        # Create a fixed constraint for each pair of linked joints
        for parent_joint, child_joint in self.joint_indices:
            # Create the constraint
            constraint_id = p.createConstraint(self.robotId, parent_joint, self.robotId, child_joint, p.JOINT_FIXED, [0,0,0.1], [0,0,0], [0,0,0])
            # If you need to store the constraint ID for later use, you can add it to a list or dictionary here
            p.changeConstraint(constraint_id, maxForce=1e20)

        # Disable the motors for all joints
        for i in range(self.n):
            maxForce = 0
            mode = p.VELOCITY_CONTROL
            p.setJointMotorControl2(self.robotId, i,
                                    controlMode=mode, force=maxForce)
        return
        
    def init_stewart(self,flag):
        # Design variables
        r_P,r_B, gama_P,gama_B = self.design_variable
        self.clf = ik(r_P,r_B,gama_P,gama_B)
        # compute the leg length for the initial position
        translation = np.array([0, 0, 0])       # translation in meters
        rotation = np.array([0, 0, 0])         # rotation in degrees
        self.l  = self.clf.solve(translation, rotation)
        # print("leg length",self.l)
        if flag:
            translation = np.array([0, 0, 0.09])
            leg_1  = self.clf.solve(translation, rotation)
            # print("leg length",self.l,leg_1)
            self.linear_actuator(leg_1-self.l, 1)
            time.sleep(1.)
        return 
    
    # Motor driver function
    def linear_actuator(self, linear_distance, actuation_duration):
        # Calculate the step size and the PWM parameters
        frequency = 50                                   # 50 Hz
        self.max_force = 250                                # 250 N linear actuation force
        steps = int(actuation_duration * frequency)  # pwm steps
        pwm_period = 1.0 / frequency                  # pwm period
        duty_cycle = 0.5                            # 50% duty cycle
        pwm_high_time = pwm_period * duty_cycle 
        actuation_step = np.array([l / steps for l in linear_distance]) # actuation step
        # Actuate the linear actuators
        for i in range(steps):
            # Read the pwm signal
            pwm_signal = i * pwm_period % pwm_period < pwm_high_time
            # pwm signal is high then actuate the linear actuator
            if pwm_signal:
                # print("PWM signal is high")
                # Calculate the target position
                target_position = actuation_step * i + self.prev_target

            else:
                # print("PWM signal is low")
                # Hold the position
                target_position = np.array([p.getJointState(self.robotId, self.actuator_indices[j])[0]
                                            for j in range(len(self.actuator_indices))])
            
            # Actuate the linear actuator
            p.setJointMotorControlArray(self.robotId, self.actuator_indices, 
                                        controlMode = p.POSITION_CONTROL, 
                                        targetPositions = target_position, 
                                        forces=self.max_force*np.ones(len(self.actuator_indices)))
            time.sleep(1./(frequency))
            p.stepSimulation()
            
        self.prev_target = np.array(actuation_step) * i+ self.prev_target
        # print("actuation step",self.prev_target)
        return 
    def reset_position(self):
        time.sleep(1)
        # # Reset the position of the robot
        self.linear_actuator(-self.prev_target, 1)
        p.setJointMotorControlArray(self.robotId, self.actuator_indices, 
                                        controlMode = p.POSITION_CONTROL, 
                                        targetPositions = np.zeros(len(self.actuator_indices)), 
                                        forces=self.max_force*np.ones(len(self.actuator_indices)))
        self.current_pose = np.zeros(6)
        return
       
    def start_simmulation(self, data, simulation=False, flag = True):
        self.set_env()         # set the environment
        self.set_constraints()  # set the constraints
        if simulation:
            logging_id = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "simulation.mp4")
        self.init_stewart(flag)  # initialize the stewart platform
        for i in data:
            trans,rot,t = i
            l = self.clf.solve(trans, rot) # compute the leg length
            dl = l-self.l                    # compute the leg actuation distance
            self.linear_actuator(dl, t)         # actuate the linear actuator
                       # reset the position
        self.reset_position()
        cubePos, cubeOrn = p.getBasePositionAndOrientation(self.robotId)
        for i in range (100):
            p.stepSimulation()
            time.sleep(1./240.)
            cubePos, cubeOrn = p.getBasePositionAndOrientation(self.robotId)
        
        # print(cubePos,cubeOrn)
        # Stop recording the simulation
        if simulation:
            p.stopStateLogging(logging_id)
        p.disconnect()
        return
    
    def fit(self, datas, flag, simulation=False):
        self.set_env()         # set the environment
        self.set_constraints()  # set the constraints
        
        if simulation:
            logging_id = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "simulation.mp4")
        
        for i,data in enumerate(datas):
            self.init_stewart(flag[i])  # initialize the stewart platform
            # print(data)
            for i in data:
                trans,rot,t = i
                l = self.clf.solve(trans, rot) # compute the leg length
                dl = l-self.l                    # compute the leg actuation distance
                self.linear_actuator(dl, t)         # actuate the linear actuator
            
            # reset the position
            self.reset_position()
        cubePos, cubeOrn = p.getBasePositionAndOrientation(self.robotId)
        for i in range (100):
            p.stepSimulation()
            time.sleep(1./240.)
            cubePos, cubeOrn = p.getBasePositionAndOrientation(self.robotId)
        
        # print(cubePos,cubeOrn)
        # Stop recording the simulation
        if simulation:
            p.stopStateLogging(logging_id)
        p.disconnect()
        return

    # --- API: Set pose directly ---
    def set_pose(self, pitch, roll, yaw, sway, surge, heave, duration=1.0):
        """
        Move the platform to the given pose (degrees, mm) over the specified duration.
        pitch, roll, yaw: degrees
        sway, surge, heave: mm
        """
        self.set_env()
        self.set_constraints()
        self.init_stewart(flag=False)
        translation = np.array([sway, surge, heave]) / 1000.0  # mm to meters
        rotation = np.array([pitch, roll, yaw])
        l = self.clf.solve(translation, rotation)
        dl = l - self.l
        self.linear_actuator(dl, duration)
        self.current_pose = np.array([pitch, roll, yaw, sway, surge, heave])
        return

    # --- API: Get current pose ---
    def get_pose(self):
        """
        Return the current pose as [pitch, roll, yaw, sway, surge, heave].
        """
        return self.current_pose.tolist()

    # --- API: Run sine wave on each DOF ---
    def run_sine(self, amplitudes, frequencies, phases, duration=5.0, dt=0.02):
        """
        amplitudes: [pitch, roll, yaw, sway, surge, heave] (deg or mm)
        frequencies: [Hz] for each DOF
        phases: [deg] for each DOF
        duration: total time to run
        dt: timestep
        """
        self.set_env()
        self.set_constraints()
        self.init_stewart(flag=False)
        t0 = time.time()
        t = 0.0
        while t < duration:
            pose = []
            for i in range(6):
                # phase in radians
                phase_rad = np.deg2rad(phases[i])
                value = amplitudes[i] * np.sin(2 * np.pi * frequencies[i] * t + phase_rad)
                pose.append(value)
            # pose: [pitch, roll, yaw, sway, surge, heave]
            self.set_pose(*pose, duration=dt)
            t = time.time() - t0
        return

    # --- API: Execute command dict (single-step, sine, etc.) ---
    def execute_command(self, command):
        """
        command: dict with keys 'type' (single, sine), and relevant params
        Example for single-step:
            {'type': 'single', 'pose': [pitch, roll, yaw, sway, surge, heave], 'duration': 1.0}
        Example for sine:
            {'type': 'sine', 'amplitudes': [...], 'frequencies': [...], 'phases': [...], 'duration': 5.0}
        """
        if command['type'] == 'single':
            self.set_pose(*command['pose'], duration=command.get('duration', 1.0))
        elif command['type'] == 'sine':
            self.run_sine(command['amplitudes'], command['frequencies'], command['phases'], duration=command.get('duration', 5.0))
        elif command['type'] == 'reset':
            self.reset_position()
        else:
            raise ValueError(f"Unknown command type: {command['type']}")

    # --- Simulation loop for continuous update (for future extension) ---
    def simulation_loop(self, update_fn, duration=10.0, dt=0.02):
        """
        update_fn: function that returns [pitch, roll, yaw, sway, surge, heave] at each timestep
        duration: total time to run
        dt: timestep
        """
        self.set_env()
        self.set_constraints()
        self.init_stewart(flag=False)
        t0 = time.time()
        t = 0.0
        while t < duration:
            pose = update_fn(t)
            self.set_pose(*pose, duration=dt)
            t = time.time() - t0
        return