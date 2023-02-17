import numpy as np
import pybullet
from pybullet_utils import bullet_client
import time
import json
import importlib
import meshcat
from pathlib import Path
import asyncio


class Simulator:
    def __init__(
                self,
                display=True,
                display_pybullet=False,
                seed=None,
                width=640,
                height=480,
                damping=0.,
                tau_max=1.,
                station_velocity=-0.5,
                bumpy=True,
                dt=0.01,
            ):
        
        # Random number generator
        self.rng = np.random.default_rng(seed)

        # Size of display
        self.width = width
        self.height = height

        # Time step
        self.dt = dt

        # Which station model to use
        self.bumpy = bumpy
        if self.bumpy:
            self.station_filename = 'bumpy-station.urdf'
        else:
            self.station_filename = 'station.urdf'

        # Other parameters
        # - Passed
        self.damping = damping
        self.tau_max = tau_max
        self.station_velocity = station_velocity
        # - Hard-coded
        self.wheel_radius = 0.325
        self.wheel_base = 0.7
        self.station_radius = 20.
        # - Variable
        self.centerline = 0.

        # Connect to and configure pybullet
        self.display_pybullet = display_pybullet
        self.display_meshcat = display
        if self.display_pybullet:
            self.bullet_client = bullet_client.BulletClient(
                connection_mode=pybullet.GUI,
                options=f'--width={width} --height={height}',
            )
            self.bullet_client.configureDebugVisualizer(
                self.bullet_client.COV_ENABLE_GUI, 0,
                lightPosition=[10., 10., 10.],
            )
        else:
            self.bullet_client = bullet_client.BulletClient(
                connection_mode=pybullet.DIRECT,
            )
        self.bullet_client.setPhysicsEngineParameter(
            fixedTimeStep=self.dt,
            numSubSteps=4,
            restitutionVelocityThreshold=0.05,
            enableFileCaching=0,
        )

        # Load station
        self.station_id = self.bullet_client.loadURDF(
            str(Path(f'./urdf/{self.station_filename}')),
            basePosition=np.array([0., 0., 0.]),
            baseOrientation=self.bullet_client.getQuaternionFromEuler([np.pi / 2, 0., 0.]),
            flags=(self.bullet_client.URDF_USE_IMPLICIT_CYLINDER  |
                   self.bullet_client.URDF_USE_INERTIA_FROM_FILE  ),
        )

        # Enable velocity control to keep station spinning at fixed rate
        self.bullet_client.setJointMotorControl2(
            self.station_id,
            0,
            self.bullet_client.VELOCITY_CONTROL,
            targetVelocity=self.station_velocity,
            force=1000.,
        )
        
        # Load robot
        self.robot_id = self.bullet_client.loadURDF(
            str(Path('./urdf/segbot.urdf')),
            basePosition=np.array([0., 0., -self.station_radius + (2 * self.wheel_radius) + 0.01]),
            baseOrientation=self.bullet_client.getQuaternionFromEuler([0., 0., 0.]),
            flags=(self.bullet_client.URDF_USE_IMPLICIT_CYLINDER  |
                   self.bullet_client.URDF_USE_INERTIA_FROM_FILE  ))
        
        # Create a dictionary that maps joint names to joint indices and
        # link names to link indices
        self.joint_map = {}
        self.link_map = {}
        for joint_id in range(self.bullet_client.getNumJoints(self.robot_id)):
            joint_name = self.bullet_client.getJointInfo(self.robot_id, joint_id)[1].decode('UTF-8')
            link_name = self.bullet_client.getJointInfo(self.robot_id, joint_id)[12].decode('UTF-8')
            self.joint_map[joint_name] = joint_id
            self.link_map[link_name] = joint_id

        # Create an array with the index of each joint we care about
        self.joint_names = [
            'chassis_to_left_wheel',
            'chassis_to_right_wheel',
        ]
        self.num_joints = len(self.joint_names)
        self.joint_ids = np.array([self.joint_map[joint_name] for joint_name in self.joint_names])

        # Set damping of all joints to given value
        for id in self.joint_ids:
            self.bullet_client.changeDynamics(self.robot_id, id, jointDamping=damping)

        # Set contact and damping parameters
        for object_id in [self.robot_id, self.station_id]:
            for joint_id in range(-1, self.bullet_client.getNumJoints(object_id)):
                self.bullet_client.changeDynamics(
                    object_id,
                    joint_id,
                    lateralFriction=1.0,
                    spinningFriction=0.0,
                    rollingFriction=0.0,
                    restitution=0.5,
                    contactDamping=-1,
                    contactStiffness=-1,
                    linearDamping=0.,
                    angularDamping=0.,
                )

        # Disable velocity control on each robot joint so we can use torque control
        self.bullet_client.setJointMotorControlArray(
            self.robot_id,
            self.joint_ids,
            self.bullet_client.VELOCITY_CONTROL,
            forces=np.zeros(self.num_joints),
        )

        # Initialize meshcat if necessary
        if self.display_meshcat:
            self.meshcat_init()

        # Set default camera view
        self.camera_backview()
    
    def get_sensor_measurements(self):
        """
        The measurements are:

            lateral error
            heading error
            forward speed
            turning rate
            pitch angle
            pitch rate

        They are computed assuming both wheels roll without slipping.
        """

        # Position of each wheel (center)
        link_states = self.bullet_client.getLinkStates(self.robot_id, self.joint_ids)
        pl = np.array(link_states[0][0])
        pr = np.array(link_states[1][0])
        pc = 0.5 * (pr + pl)

        # Velocity of each wheel
        joint_states = self.bullet_client.getJointStates(self.robot_id, self.joint_ids)
        q = np.zeros([self.num_joints])
        v = np.zeros_like(q)
        for i in range(self.num_joints):
            q[i] = joint_states[i][0]
            v[i] = joint_states[i][1]
        vl = v[0] * self.wheel_radius
        vr = v[1] * self.wheel_radius

        # Lateral error (positive when located too far to left)
        lateral_error = pc[1]

        # Heading error (positive when turned too far to left)
        # - local reference frame
        #  - frame 1 moves with robot around station
        #  - frame 0 is the world frame
        theta = - np.arctan2(pc[0], -pc[2])
        R_1in0 = np.array([[np.cos(theta), 0., np.sin(theta)],
                           [0., 1., 0.],
                           [-np.sin(theta), 0., np.cos(theta)]])
        # - vector from left wheel to right wheel
        a_in0 = pr - pl
        a_in1 = R_1in0.T @ a_in0
        # - heading error
        heading_error = np.arctan2(a_in1[0], -a_in1[1])

        # Forward speed and turning rate
        forward_speed = (vr + vl) / 2.0
        turning_rate = (vr - vl) / np.linalg.norm(pr - pl)

        # Position, orientation, and angular velocity of chassis
        pos, ori = self.bullet_client.getBasePositionAndOrientation(self.robot_id)
        vel = self.bullet_client.getBaseVelocity(self.robot_id)
        R_body_in_world = np.reshape(np.array(self.bullet_client.getMatrixFromQuaternion(ori)), (3, 3))
        w_in_world = np.reshape(np.array(vel[1]), (3, 1))
        w_in_body = R_body_in_world.T @ w_in_world
        R_body_in_1 = R_1in0.T @ R_body_in_world
        pitch_angle = np.arctan2(
            -R_body_in_1[2, 0],
            np.sqrt(R_body_in_1[2, 1]**2 + R_body_in_1[2, 2]**2)
        )
        
        # Pitch angle and pitch rate
        pitch_rate = w_in_body[1, 0] + self.station_velocity

        return lateral_error + self.centerline, heading_error, forward_speed, turning_rate, pitch_angle, pitch_rate
    
    def set_actuator_commands(
                self,
                desired_right_wheel_torque,
                desired_left_wheel_torque
            ):
        
        if not np.isscalar(desired_right_wheel_torque):
            raise Exception('right_wheel_torque must be a scalar')
        
        if not np.isscalar(desired_left_wheel_torque):
            raise Exception('left_wheel_torque must be a scalar')

        right_wheel_torque = np.clip(desired_right_wheel_torque, -self.tau_max, self.tau_max)
        left_wheel_torque = np.clip(desired_left_wheel_torque, -self.tau_max, self.tau_max)
        self.set_joint_torque(np.array([left_wheel_torque, right_wheel_torque]))
        return right_wheel_torque, left_wheel_torque
    
    def set_joint_torque(self, tau):
        if tau.shape[0] != self.num_joints:
            raise Exception('tau must be same length as number of joints')
        zero_gains = tau.shape[0] * (0.,)
        self.bullet_client.setJointMotorControlArray(
            self.robot_id,
            self.joint_ids,
            self.bullet_client.TORQUE_CONTROL,
            forces=tau,
            positionGains=zero_gains,
            velocityGains=zero_gains,
        )

    def reset(
            self,
            initial_speed=0.,
            initial_lateral_error=0.,
            initial_heading_error=0.,
            initial_pitch=0.,
            station_velocity=-0.5,
        ):

        # Station
        self.station_velocity = station_velocity
        self.bullet_client.resetJointState(
            self.station_id,
            0,
            0.,
            self.station_velocity,
        )
        self.bullet_client.setJointMotorControl2(
            self.station_id,
            0,
            self.bullet_client.VELOCITY_CONTROL,
            targetVelocity=self.station_velocity,
            force=1000.,
        )

        # Robot
        # - position and orientation
        r = self.station_radius - ((1 + np.cos(initial_pitch)) * self.wheel_radius) - 0.01
        theta = 0.
        pos = [np.sin(theta) * r, initial_lateral_error, np.cos(theta) * -r]
        ori = [0., initial_pitch - theta, initial_heading_error]
        self.bullet_client.resetBasePositionAndOrientation(
            self.robot_id,
            pos,
            self.bullet_client.getQuaternionFromEuler(ori)
        )
        # - joint angles and velocities
        angvel_wheels = initial_speed / self.wheel_radius
        for i, joint_id in enumerate(self.joint_ids):
            self.bullet_client.resetJointState(self.robot_id, joint_id, 0., angvel_wheels)
        # - linear and angular velocity
        self.bullet_client.resetBaseVelocity(
            self.robot_id,
            linearVelocity=[
                initial_speed + ((self.station_radius - (2 * self.wheel_radius)) * (self.station_velocity)),
                0.,
                0.,
            ],
            angularVelocity=[
                0.,
                -self.station_velocity,
                0.,
            ],
        )
        # - laps (in units of revolutions)
        self.laps = 0.

        # Update camera and display
        self._update_camera()
        self._update_display()
    
    async def run(
            self,
            controller,
            max_time=5.0,
            data_filename=None,
            video_filename=None,
            print_debug=False,
            slider=None,
        ):

        self.data = {
            't': [],
            'lateral_error': [],
            'heading_error': [],
            'forward_speed': [],
            'turning_rate': [],
            'pitch_angle': [],
            'pitch_rate': [],
            'right_wheel_torque': [],
            'right_wheel_torque_command': [],
            'left_wheel_torque': [],
            'left_wheel_torque_command': [],
            'laps': [],
        }
        self.variables_to_log = getattr(controller, 'variables_to_log', [])
        for key in self.variables_to_log:
            if key in self.data.keys():
                raise Exception(f'Trying to log duplicate variable {key} (choose a different name)')
            self.data[key] = []

        # Always start from zero time
        self.t = 0.
        self.time_step = 0
        self.max_time_steps = 1 + int(max_time / self.dt)
        self.start_time = time.time()

        if video_filename is not None:
            # Import imageio
            imageio = importlib.import_module('imageio')

            # Open video
            fps = int(1 / self.dt)
            if print_debug:
                print(f'Creating a video with name {video_filename} and fps {fps}')
            w = imageio.get_writer(video_filename,
                                   format='FFMPEG',
                                   mode='I',
                                   fps=fps)

            # Add first frame to video
            rgba = self.snapshot()
            w.append_data(rgba)

        while True:
            all_done = await self.step(controller, slider)

            if self.display_meshcat or self.display_pybullet:
                self._update_camera()

            if self.display_meshcat:
                self.meshcat_update()

            if video_filename is not None:
                if self.time_step % 100 == 0:
                    if print_debug:
                        print(f' {self.time_step} / {self.max_time_steps}')

                # Add frame to video
                rgba = self.snapshot()
                w.append_data(rgba)

            if all_done:
                break

            if (self.max_time_steps is not None) and (self.time_step == self.max_time_steps):
                break

        if video_filename is not None:
            # Close video
            w.close()

        if data_filename is not None:
            with open(data_filename, 'w') as f:
                json.dump(self.data, f)

        stop_time = time.time()
        stop_time_step = self.time_step

        elapsed_time = stop_time - self.start_time
        elapsed_time_steps = stop_time_step
        if (elapsed_time > 0) and print_debug:
            print(f'Simulated {elapsed_time_steps} time steps in {elapsed_time:.4f} seconds ({(elapsed_time_steps / elapsed_time):.4f} time steps per second)')

        # convert lists to numpy arrays
        data = self.data.copy()
        for key in data.keys():
            data[key] = np.array(data[key])

        return data

    def update_laps(self):
        # Largest previous angle
        a_pre = self.laps * (2 * np.pi)

        # Current angle
        link_states = self.bullet_client.getLinkStates(self.robot_id, self.joint_ids)
        pl = np.array(link_states[0][0])
        pr = np.array(link_states[1][0])
        pc = 0.5 * (pr + pl)
        theta_robot = np.arctan2(pc[0], -pc[2])
        theta_station = self.bullet_client.getJointState(self.station_id, 0)[0]
        a_cur = (theta_robot - theta_station) % (2 * np.pi)

        # Angle increment (if within threshold, increase laps)
        a_inc = (a_cur - a_pre) % (2 * np.pi)
        if (a_inc > 0) and (a_inc < 0.1):
            self.laps = (a_pre + a_inc) / (2 * np.pi)

    async def step(self, controller, slider):
        # Never stop early
        all_done = False

        # Get the current time
        self.t = self.time_step * self.dt

        # Update laps
        self.update_laps()

        # Update centerline
        if slider is None:
            self.centerline = 0.
        else:
            self.centerline = slider.value

        # Get the sensor measurements
        lateral_error, heading_error, forward_speed, turning_rate, pitch_angle, pitch_rate = self.get_sensor_measurements()

        # Get torque commands (run the controller)
        right_wheel_torque_command, left_wheel_torque_command = controller.run(
            self.t,
            lateral_error,
            heading_error,
            forward_speed,
            turning_rate,
            pitch_angle,
            pitch_rate,
        )

        # Apply the torque commands
        right_wheel_torque, left_wheel_torque = self.set_actuator_commands(
            right_wheel_torque_command,
            left_wheel_torque_command,
        )

        # Log data
        self.data['t'].append(self.t)
        self.data['lateral_error'].append(lateral_error)
        self.data['heading_error'].append(heading_error)
        self.data['forward_speed'].append(forward_speed)
        self.data['turning_rate'].append(turning_rate)
        self.data['pitch_angle'].append(pitch_angle)
        self.data['pitch_rate'].append(pitch_rate)
        self.data['right_wheel_torque'].append(right_wheel_torque)
        self.data['right_wheel_torque_command'].append(right_wheel_torque_command)
        self.data['left_wheel_torque'].append(left_wheel_torque)
        self.data['left_wheel_torque_command'].append(left_wheel_torque_command)
        self.data['laps'].append(self.laps)
        for key in self.variables_to_log:
            val = getattr(controller, key, np.nan)
            if not np.isscalar(val):
                val = val.flatten().tolist()
            self.data[key].append(val)

        # Try to stay real-time
        if self.display_pybullet or self.display_meshcat:
            t = self.start_time + (self.dt * (self.time_step + 1))
            time_to_wait = t - time.time()
            await asyncio.sleep(time_to_wait)
            # while time_to_wait > 0:
            #     time.sleep(0.9 * time_to_wait)
            #     time_to_wait = t - time.time()

        # Take a simulation step
        self.bullet_client.stepSimulation()

        # Increment time step
        self.time_step += 1

        return all_done

    def pybullet_snapshot(self):
        link_states = self.bullet_client.getLinkStates(self.robot_id, self.joint_ids)
        pl = np.array(link_states[0][0])
        pr = np.array(link_states[1][0])
        pc = 0.5 * (pr + pl)
        theta = np.arctan2(pc[0], -pc[2])
        r = self.station_radius - (2 * self.wheel_radius)
        pos = [np.sin(theta) * r, 0., np.cos(theta) * -r]

        if self.view == 'sideview':
            (dist, yaw, pitch) = (3., 0., 0.)
            view_matrix = self.bullet_client.computeViewMatrixFromYawPitchRoll(pos, dist, yaw, -pitch, 0., 2)
        elif self.view == 'wideview':
            (dist, yaw, pitch) = (35., 0., 0.)
            view_matrix = self.bullet_client.computeViewMatrixFromYawPitchRoll([0., 0., 0.], dist, yaw, -pitch, 0., 2)
        elif self.view == 'backview':
            (dist, yaw, pitch) = (3., -np.rad2deg(theta) + 90, 0.)
            view_matrix = self.bullet_client.computeViewMatrixFromYawPitchRoll(pos, dist, yaw, -pitch, 0., 1)
        else:
            raise Exception('invalid camera view')
        
        aspect = self.width / self.height
        projection_matrix = self.bullet_client.computeProjectionMatrixFOV(fov=90, aspect=aspect, nearVal=0.01, farVal=100.0)
        im = self.bullet_client.getCameraImage(
            self.width, self.height,
            viewMatrix=view_matrix,
            projectionMatrix=projection_matrix,
            renderer=self.bullet_client.ER_BULLET_HARDWARE_OPENGL,
            shadow=1,
            lightDirection=[10., 10., 10.],
        )
        rgba = im[2]
        return rgba
    
    def meshcat_snapshot(self):
        # Get image from visualizer
        rgba = np.asarray(self.vis.get_image())

        # Shrink width and height to be multiples of 16
        height, width, channels = rgba.shape
        m = 16
        return np.ascontiguousarray(rgba[
            :(m * np.floor(height / m).astype(int)),
            :(m * np.floor(width / m).astype(int)),
            :,
        ])
    
    def snapshot(self):
        if self.display_meshcat:
            return self.meshcat_snapshot()
        else:
            return self.pybullet_snapshot()

    def _update_display(self):
        if self.display_pybullet:
            # hack to get GUI to update on MacOS
            time.sleep(0.01)
            keys = self.bullet_client.getKeyboardEvents()
        
        if self.display_meshcat:
            self.meshcat_update()

    def _update_camera(self):
        if self.display_pybullet or self.display_meshcat:
            if self.view == 'backview':
                self._update_backview()
            elif self.view == 'sideview':
                self._update_sideview()
            elif self.view == 'wideview':
                pass
            else:
                raise Exception('invalid camera view')

    def camera_backview(self):
        self.view = 'backview'
        if not (self.display_meshcat or self.display_pybullet):
            return
        
        if self.display_pybullet:
            self.bullet_client.configureDebugVisualizer(self.bullet_client.COV_ENABLE_Y_AXIS_UP, 1)
        
        self._update_backview()
        self._update_display()
    
    def _update_backview(self):
        if not (self.display_meshcat or self.display_pybullet):
            return
        
        link_states = self.bullet_client.getLinkStates(self.robot_id, self.joint_ids)
        pl = np.array(link_states[0][0])
        pr = np.array(link_states[1][0])
        pc = 0.5 * (pr + pl)
        theta = np.arctan2(pc[0], -pc[2])
        r = self.station_radius - (2 * self.wheel_radius)
        pos = [np.sin(theta) * r, 0., np.cos(theta) * -r]

        if self.display_meshcat:
            self.camera_distance = 3.
            self.camera_yaw = 0.
            self.camera_pitch = -np.rad2deg(theta)
            self.camera_target = pos
            self.meshcat_camera()
        
        if self.display_pybullet:
            self.bullet_client.resetDebugVisualizerCamera(3., -np.rad2deg(theta) + 90, 0, pos)
    
    def camera_sideview(self):
        self.view = 'sideview'
        if not (self.display_meshcat or self.display_pybullet):
            return

        if self.display_pybullet:
            self.bullet_client.configureDebugVisualizer(self.bullet_client.COV_ENABLE_Y_AXIS_UP, 0)
        
        self._update_sideview()
        self._update_display()

    def _update_sideview(self):
        if not (self.display_meshcat or self.display_pybullet):
            return

        link_states = self.bullet_client.getLinkStates(self.robot_id, self.joint_ids)
        pl = np.array(link_states[0][0])
        pr = np.array(link_states[1][0])
        pc = 0.5 * (pr + pl)
        theta = np.arctan2(pc[0], -pc[2])
        r = self.station_radius - (2 * self.wheel_radius)
        pos = [np.sin(theta) * r, 0., np.cos(theta) * -r]

        if self.display_meshcat:
            self.camera_distance = 3.
            self.camera_yaw = 90.
            self.camera_pitch = -np.rad2deg(theta)
            self.camera_target = pos
            self.meshcat_camera()
        
        if self.display_pybullet:
            self.bullet_client.resetDebugVisualizerCamera(3., 0, 0, pos)
    
    def camera_wideview(self):
        self.view = 'wideview'
        if not (self.display_meshcat or self.display_pybullet):
            return
        
        if self.display_pybullet:
            self.bullet_client.configureDebugVisualizer(self.bullet_client.COV_ENABLE_Y_AXIS_UP, 0)
            self.bullet_client.resetDebugVisualizerCamera(35., 0, 0, [0., 0., 0.])
        
        if self.display_meshcat:
            self.camera_distance = 35.
            self.camera_yaw = 90.
            self.camera_pitch = 0.
            self.camera_target = [0., 0., 0.]
            self.meshcat_camera()
        
        self._update_display()
    
    def _wxyz_from_xyzw(self, xyzw):
        return np.roll(xyzw, 1)

    def _convert_color(self, rgba):
        color = int(rgba[0] * 255) * 256**2 + int(rgba[1] * 255) * 256 + int(rgba[2] * 255)
        opacity = rgba[3]
        transparent = opacity != 1.0
        return {
            'color': color,
            'opacity': opacity,
            'transparent': transparent,
        }

    def meshcat_camera(self):
        self.vis['/Cameras/default'].set_transform(
            meshcat.transformations.compose_matrix(
                angles=[
                    0.,
                    0.,
                    np.deg2rad(self.camera_yaw - 180),
                ],
                translate=[0., 0., 0.],
            )
        )
        self.vis['/Cameras/default/rotated/<object>'].set_property(
            'position', [self.camera_distance, 0., 0.],
        )
        self.vis['/Cameras/default/rotated/<object>'].set_property(
            'fov', 90,
        )
    
    def meshcat_init(self):
        # Create a visualizer
        self.vis = meshcat.Visualizer().open()

        # Make sure everything has been deleted from the visualizer
        self.vis.delete()

        # Add station
        self.station_links = []
        for s in self.bullet_client.getVisualShapeData(self.station_id):
            link_id = s[1]
            stl_filename = s[4].decode('UTF-8')
            scale = s[3]
            color = self._convert_color(s[7])

            # It is only possible to ask pybullet for the name of non-base links
            is_base = (link_id == -1)
            if is_base:
                link_name = 'base'
            else:
                joint_info = self.bullet_client.getJointInfo(self.station_id, link_id)
                link_name = joint_info[12].decode('UTF-8')

            self.station_links.append({
                'name': link_name,
                'id': link_id,
                'scale': scale,
                'is_base': is_base,
            })

            self.vis['station'][link_name].set_object(
                meshcat.geometry.StlMeshGeometry.from_file(stl_filename),
                meshcat.geometry.MeshPhongMaterial(
                    color=color['color'],
                    transparent=color['transparent'],
                    opacity=color['opacity'],
                    reflectivity=0.8,
                )
            )
        
        # Add robot
        self.robot_links = []
        for s in self.bullet_client.getVisualShapeData(self.robot_id):
            link_id = s[1]
            stl_filename = s[4].decode('UTF-8')
            scale = s[3]
            color = self._convert_color(s[7])

            # It is only possible to ask pybullet for the name of non-base links
            is_base = (link_id == -1)
            if is_base:
                link_name = 'base'
            else:
                joint_info = self.bullet_client.getJointInfo(self.robot_id, link_id)
                link_name = joint_info[12].decode('UTF-8')

            self.robot_links.append({
                'name': link_name,
                'id': link_id,
                'scale': scale,
                'is_base': is_base,
            })

            self.vis['robot'][link_name].set_object(
                meshcat.geometry.StlMeshGeometry.from_file(stl_filename),
                meshcat.geometry.MeshPhongMaterial(
                    color=color['color'],
                    transparent=color['transparent'],
                    opacity=color['opacity'],
                    reflectivity=0.8,
                )
            )
        
        # Add centerline
        color = self._convert_color([255 / 255, 209 / 255, 37 / 255, 1.])
        self.vis['centerline'].set_object(
            meshcat.geometry.Sphere(0.1),
            meshcat.geometry.MeshPhongMaterial(
                color=color['color'],
                transparent=color['transparent'],
                opacity=color['opacity'],
                reflectivity=0.8,
            )
        )
        
        # Turn off grid
        self.vis['/Grid'].set_property('visible', False)
        
    def meshcat_update(self):

        #
        # TODO   Use getLinkStates instead of getLinkState to make
        #        this function more efficient. This is even dumber
        #        because I use getLinkStates for the centerline.
        #

        # Move scene to move camera, because meshcat-python does not
        # fully support camera control
        T_cam_in_world = meshcat.transformations.compose_matrix(
            angles=[
                0.,
                np.deg2rad(self.camera_pitch),
                0.,
            ],
            translate=self.camera_target,
        )
        T_world_in_cam = meshcat.transformations.inverse_matrix(T_cam_in_world)

        # Set pose of station links
        for link in self.station_links:
            if link['is_base']:
                pos, ori = self.bullet_client.getBasePositionAndOrientation(self.station_id)
            else:
                link_state = self.bullet_client.getLinkState(self.station_id, link['id'])
                pos = link_state[4]
                ori = link_state[5]

            S = np.diag(np.concatenate((link['scale'], [1.0])))
            T = meshcat.transformations.quaternion_matrix(self._wxyz_from_xyzw(ori))
            T[:3, 3] = np.array(pos)[:3]
            self.vis['station'][link['name']].set_transform(T_world_in_cam @ T @ S)

        # Set pose of robot links
        for link in self.robot_links:
            if link['is_base']:
                pos, ori = self.bullet_client.getBasePositionAndOrientation(self.robot_id)
            else:
                link_state = self.bullet_client.getLinkState(self.robot_id, link['id'])
                pos = link_state[4]
                ori = link_state[5]
            
            S = np.diag(np.concatenate((link['scale'], [1.0])))
            T = meshcat.transformations.quaternion_matrix(self._wxyz_from_xyzw(ori))
            T[:3, 3] = np.array(pos)[:3]
            self.vis['robot'][link['name']].set_transform(T_world_in_cam @ T @ S)
        
        # Set pose of centerline
        link_states = self.bullet_client.getLinkStates(self.robot_id, self.joint_ids)
        pl = np.array(link_states[0][0])
        pr = np.array(link_states[1][0])
        pc = 0.5 * (pr + pl)
        theta = np.arctan2(pc[0], -pc[2])
        r = self.station_radius
        pos = [np.sin(theta) * r, -self.centerline, np.cos(theta) * -r]

        self.vis['centerline'].set_transform(
            T_world_in_cam @ meshcat.transformations.translation_matrix(pos),
        )
    
    
