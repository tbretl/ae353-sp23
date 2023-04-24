import numpy as np
import pybullet
from pybullet_utils import bullet_client
import time
import json
import meshcat
from pathlib import Path
import time
from scipy import linalg
import importlib
import pkgutil
import traceback
import contextlib
import io
from scipy.spatial.transform import Rotation


class Simulator:

    def __init__(
            self,
            display=True,
            display_pybullet=False,
            seed=None,
            width=640,
            height=480,
        ):

        # Random number generator
        self.rng = np.random.default_rng(seed)

        # Size of display
        self.width = width
        self.height = height

        # Time step
        self.dt = 0.04

        # Whether or not to error on controller print, timeout, or inactivity
        self.error_on_print = True
        self.error_on_timeout = True
        self.error_on_inactive = True
        self.max_controller_run_time=1e-2
        self.max_controller_reset_time=1e0
        self.max_controller_init_time=1e0
        self.max_controller_load_time=5e0
        self.max_run_time_violations=10

        # Create empty list of drones
        self.drones = []
        self.max_num_drones = 40

        # Connect to and configure pybullet
        self.display_pybullet = display_pybullet
        self.display_meshcat = display
        if self.display_pybullet:
            options = f'--width={width} --height={height}'
            self.bullet_client = bullet_client.BulletClient(
                connection_mode=pybullet.GUI,
                options=options,
            )
            self.bullet_client.configureDebugVisualizer(
                self.bullet_client.COV_ENABLE_GUI, 0,
            )
        else:
            self.bullet_client = bullet_client.BulletClient(
                connection_mode=pybullet.DIRECT,
            )
        self.bullet_client.setGravity(0, 0, -9.81)
        self.bullet_client.setPhysicsEngineParameter(
            fixedTimeStep=self.dt,
            numSubSteps=4,
            restitutionVelocityThreshold=0.05,
            enableFileCaching=0,
        )

        # Load plane
        self.plane_id = self.bullet_client.loadURDF(
            str(Path('./urdf/plane.urdf')),
            basePosition=np.array([0., 0., 0.]),
            baseOrientation=self.bullet_client.getQuaternionFromEuler([0., 0., 0.]),
            useFixedBase=1,
        )

        # Set contact parameters
        self.bullet_client.changeDynamics(self.plane_id, -1,
                lateralFriction=1.0,
                spinningFriction=0.0,
                rollingFriction=0.0,
                restitution=0.5,
                contactDamping=-1,
                contactStiffness=-1)
        
        # Initialize meshcat if necessary
        if self.display_meshcat:
            self.meshcat_init()
        
        # Load rings
        self.rings = []
        self.num_rings = 0
        self.place_rings()

        # Default camera view
        self.camera_startview()

        # Parameters that govern applied force and torque
        #
        #  motor[0]: front (+z spin)
        #  motor[1]: rear (+z spin)
        #  motor[2]: left (-z spin)
        #  motor[3]: right (-z spin)
        #
        self.l = 0.175
        self.kF = 7e-6
        self.kM = 1e-7
        self.min_spin_rate = 100 # <-- rad/s
        self.max_spin_rate = 900 # <-- rad/s
        self.s_min = self.min_spin_rate**2
        self.s_max = self.max_spin_rate**2
        self.M = linalg.inv(np.array([[0., 0., self.kF * self.l, -self.kF * self.l],
                                      [-self.kF * self.l, self.kF * self.l, 0., 0.],
                                      [-self.kM, -self.kM, self.kM, self.kM],
                                      [self.kF, self.kF, self.kF, self.kF]]))
        
        # Parameters that govern inactivity (drone needs to move
        # outside a box of at least side length activity_d in past
        # activity_t seconds)
        self.activity_d = 0.5
        self.activity_t = 5.0
        self.activity_n = 1 + int(self.activity_t / self.dt)

        # Parameters that govern out-of-bounds
        #
        #  -bounds_d <= p_x <= bounds_d
        #  -bounds_d <= p_y <= bounds_d
        #          0 <= p_z <= bounds_d
        #
        self.bounds_d = 25.

    def set_rules(self, error_on_print=True, error_on_timeout=True, error_on_inactive=True):
        self.error_on_print = error_on_print
        self.error_on_timeout = error_on_timeout
        self.error_on_inactive = error_on_inactive

    def clear_drones(self):
        self.meshcat_clear_drones()
        for drone in self.drones:
            self.bullet_client.removeBody(drone['id'])
        self.drones = []
        if self.view['type'] != 'finish':
            self.camera_startview()

    def add_drone(self, Controller, name, image):
        if self.get_drone_by_name(name) is not None:
            raise Exception(f'drone with name "{name}" already exists')
        try:
            # create instance of controller
            controller_start_time = time.time()
            if self.error_on_print:
                with contextlib.redirect_stdout(io.StringIO()) as stdout:
                    controller = Controller()
                stdout_val = stdout.getvalue()
                if stdout_val:
                    raise Exception(f'Printed the following text to stdout, which is forbidden:\n\n{stdout_val}')
            else:
                controller = Controller()
            controller_run_time = time.time() - controller_start_time
            if (controller_run_time > self.max_controller_init_time) and self.error_on_timeout:
                raise Exception(f'Init timeout exceeded: {controller_run_time} > {self.max_controller_init_time}')

            # get color
            color = controller.get_color()
            assert(len(color) == 3)
            color.append(1.)

            # get label
            if image is not None:
                texture_id = self.bullet_client.loadTexture(image)

            # load urdf
            id = self.bullet_client.loadURDF(
                str(Path('./urdf/drone.urdf')),
                basePosition=np.array([0., 0., 0.3]),
                baseOrientation=self.bullet_client.getQuaternionFromEuler([0., 0., 0.]),
                useFixedBase=0,
                flags=(self.bullet_client.URDF_USE_IMPLICIT_CYLINDER  |
                        self.bullet_client.URDF_USE_INERTIA_FROM_FILE  ),
            )
            
            # map joint names to joint indices and link names to link indices
            joint_map = {}
            link_map = {}
            link_map['base'] = -1
            for joint_id in range(self.bullet_client.getNumJoints(id)):
                joint_name = self.bullet_client.getJointInfo(id, joint_id)[1].decode('UTF-8')
                link_name = self.bullet_client.getJointInfo(id, joint_id)[12].decode('UTF-8')
                if link_name == 'base':
                    raise Exception('cannot have a non-base link named "base"')
                joint_map[joint_name] = joint_id
                link_map[link_name] = joint_id

            # apply color and label
            self.bullet_client.changeVisualShape(id, link_map['base'], rgbaColor=color)
            if image is None:
                self.bullet_client.changeVisualShape(id, link_map['screen'], rgbaColor=[1., 1., 1., 0.])
            else:
                self.bullet_client.changeVisualShape(id, link_map['screen'], rgbaColor=[1., 1., 1., 0.75], textureUniqueId=texture_id)

            # set contact parameters
            self.bullet_client.changeDynamics(
                id,
                link_map['base'],
                lateralFriction=1.0,
                spinningFriction=0.0,
                rollingFriction=0.0,
                restitution=0.5,
                contactDamping=-1,
                contactStiffness=-1)
            
            # get variables to log
            variables_to_log = getattr(controller, 'variables_to_log', [])

            # consolidate information about drone in dictionary
            drone = {
                'id': id,
                'module': None,
                'Controller': Controller,
                'name': name,
                'controller': controller,
                'joint_map': joint_map,
                'link_map': link_map,
                'variables_to_log': variables_to_log,
                'image': image,
            }

            # add drone to meshcat
            self.meshcat_add_drone(drone)

            # add drone to list of drones
            self.drones.append(drone)

            # update display
            self._update_display()
        except Exception as err:
            print(f'Failed to add {name} because of the following error:')
            print(f'\n==========\n{traceback.format_exc()}==========\n')
    
    def load_drones(self, dirname='students', no_max_num_drones=False):
        print(f'Try to import controllers from the directory "./{dirname}":')
        students = importlib.import_module(dirname)
        importlib.reload(students)
        failures = ''
        named_failures = []
        for (_, name, _) in pkgutil.iter_modules([dirname]):
            if (len(self.drones) >= self.max_num_drones) and (not no_max_num_drones):
                raise Exception(f'The simulation already has the maximum number of drones ({self.max_num_drones})')

            print(f' ./{dirname}/{name}.py')
            try:
                # check if drone by this name already exists
                if self.get_drone_by_name(name) is not None:
                    raise Exception(f'drone with name "{name}" already exists')

                # load module
                controller_start_time = time.time()
                module = importlib.import_module(f'.{name}', dirname)
                importlib.reload(module)
                controller_run_time = time.time() - controller_start_time
                if (controller_run_time > self.max_controller_load_time) and self.error_on_timeout:
                    raise Exception(f'Load timeout exceeded: {controller_run_time} > {self.max_controller_load_time}')

                # create instance of controller
                controller_start_time = time.time()
                if self.error_on_print:
                    with contextlib.redirect_stdout(io.StringIO()) as stdout:
                        controller = module.Controller()
                    stdout_val = stdout.getvalue()
                    if stdout_val:
                        raise Exception(f'Printed the following text to stdout, which is forbidden:\n\n{stdout_val}')
                else:
                    controller = module.Controller()
                controller_run_time = time.time() - controller_start_time
                if (controller_run_time > self.max_controller_init_time) and self.error_on_timeout:
                    raise Exception(f'Init timeout exceeded: {controller_run_time} > {self.max_controller_init_time}')

                # get color
                color = controller.get_color()
                assert(len(color) == 3)
                color.append(1.)

                # get label
                image = str(Path(f'./{dirname}/{name}.png'))
                texture_id = self.bullet_client.loadTexture(image)

                # load urdf
                id = self.bullet_client.loadURDF(str(Path('./urdf/drone.urdf')),
                               basePosition=np.array([0., 0., 0.3]),
                               baseOrientation=self.bullet_client.getQuaternionFromEuler([0., 0., 0.]),
                               useFixedBase=0,
                               flags=(self.bullet_client.URDF_USE_IMPLICIT_CYLINDER  |
                                      self.bullet_client.URDF_USE_INERTIA_FROM_FILE  ))

                # map joint names to joint indices and link names to link indices
                joint_map = {}
                link_map = {}
                link_map['base'] = -1
                for joint_id in range(self.bullet_client.getNumJoints(id)):
                    joint_name = self.bullet_client.getJointInfo(id, joint_id)[1].decode('UTF-8')
                    link_name = self.bullet_client.getJointInfo(id, joint_id)[12].decode('UTF-8')
                    if link_name == 'base':
                        raise Exception('cannot have a non-base link named "base"')
                    joint_map[joint_name] = joint_id
                    link_map[link_name] = joint_id

                # apply color and label
                self.bullet_client.changeVisualShape(id, link_map['base'], rgbaColor=color)
                self.bullet_client.changeVisualShape(id, link_map['screen'], rgbaColor=[1., 1., 1., 0.75], textureUniqueId=texture_id)

                # set contact parameters
                self.bullet_client.changeDynamics(id, -1,
                    lateralFriction=1.0,
                    spinningFriction=0.0,
                    rollingFriction=0.0,
                    restitution=0.5,
                    contactDamping=-1,
                    contactStiffness=-1)

                # get variables to log
                variables_to_log = getattr(controller, 'variables_to_log', [])

                # consolidate information about drone in dictionary
                drone = {
                    'id': id,
                    'module': module,
                    'Controller': module.Controller,
                    'name': name,
                    'controller': controller,
                    'joint_map': joint_map,
                    'link_map': link_map,
                    'variables_to_log': variables_to_log,
                    'image': image,
                }

                # add drone to meshcat
                self.meshcat_add_drone(drone)

                # add drone to list of drones
                self.drones.append(drone)
            except Exception as err:
                named_failures.append(name)
                failures += f'\n==========\n{dirname}/{name}.py\n==========\n{traceback.format_exc()}==========\n'
        print(f'\n\nThe following controllers failed to import and were ignored:\n{failures}')
        return named_failures
    
    def place_rings(self):
        # Hard-coded parameters (would need to change the size
        # of the ground plane and other parameters like bounds_d
        # if you wanted to allow other choices)
        num_tunnels = 3
        tunnel_separation = 5.

        # Remove all existing rings
        for ring in self.rings:
            self.bullet_client.removeBody(ring['id'])
        self.meshcat_clear_rings()
        self.rings = []
        self.num_rings = 0

        # Create a sequence of points at which to put rings (and tunnels of rings)
        len_course = (num_tunnels + 1) * tunnel_separation
        p = []
        p.append(np.array([- (len_course / 2), 0., 0.25]))
        p.append(np.array([- (len_course / 2), 0., 5.]))
        y_prev = 0.
        z_prev = 5.
        for i in range(num_tunnels):
            x = (i + 1) * tunnel_separation - (len_course / 2)
            while True:
                y = self.rng.uniform(low=-20., high=20.)
                z = self.rng.uniform(low=5., high=10.)
                p_cur = np.array([y, z])
                p_prev = np.array([y_prev, z_prev])
                p_startfinish = np.array([0., 0.])
                if (np.linalg.norm(p_cur - p_prev) > 15.) and (np.linalg.norm(p_cur - p_startfinish) > 5.):
                    break
            y_prev = y
            z_prev = z
            p.append(np.array([x, y, z]))
        p.append(np.array([(len_course / 2), 0., 5.]))
        p.append(np.array([(len_course / 2), 0., 0.25]))
        
        # Put rings (and tunnels of rings) at these points
        self.add_ring(p[0], [0., -np.pi / 2, 0.], 2.5, 0.5, 'big-ring.urdf')
        for i in range(len(p) - 2):
            self.add_tunnel(p[i], p[i + 1], p[i + 2])
        self.add_ring(p[-1], [0., np.pi / 2, 0.], 2.5, 0.5, 'big-ring.urdf')

        # Set contact parameters for all rings
        for ring in self.rings:
            self.bullet_client.changeDynamics(ring['id'], -1,
                    lateralFriction=1.0,
                    spinningFriction=0.0,
                    rollingFriction=0.0,
                    restitution=0.5,
                    contactDamping=-1,
                    contactStiffness=-1)
        
        # Add all rings to meshcat
        self.meshcat_add_rings()

    def _normalize(self, v):
        return v / np.linalg.norm(v)

    def _Rx(self, a):
        return np.array([[1., 0., 0.],
                         [0., np.cos(a), -np.sin(a)],
                         [0., np.sin(a), np.cos(a)]])
    
    def _Ry(self, a):
        return np.array([[np.cos(a), 0., np.sin(a)],
                                [0., 1., 0.],
                                [-np.sin(a), 0., np.cos(a)]])
    def _Rz(self, a):
        return np.array([[np.cos(a), -np.sin(a), 0.],
                                [np.sin(a), np.cos(a), 0.],
                                [0., 0., 1.]])

    def add_tunnel(self, p1, p2, p3):
        # Parameters
        r = 2.25
        d_psi = (2 * np.pi) / 24
        extra_n = 0
        extra_d_y = 0.35

        # Get arc frame (o_s, R_s)
        v12 = self._normalize(p2 - p1)
        v23 = self._normalize(p3 - p2)
        x_s = v12
        z_s = self._normalize(np.cross(v12, v23))
        y_s = np.cross(z_s, x_s)
        R_s = np.column_stack([x_s, y_s, z_s])
        theta = 0.5 * np.arccos(np.dot(-v12, v23))
        a = r / np.tan(theta)
        o_s = p2 - a * v12

        # Add rings before arc
        for i in range(-extra_n, 0):
            x = extra_d_y * i
            y = 0.
            p = np.array([x, y, 0.])
            R = np.eye(3)
            p_w = o_s + R_s @ p
            R_w = R_s @ R
            q_w = Rotation.from_matrix(R_w).as_quat()
            self.add_ring(
                p_w,
                q_w,
                1.00,
                0.25,
                'ring.urdf',
            )
            self.num_rings += 1

        # Add rings along arc
        max_psi = np.pi - 2 * theta
        for i in range(0, 1 + int(max_psi / d_psi)):
            psi = i * d_psi
            x = r * np.sin(psi)
            y = r * (1 -  np.cos(psi))
            p = np.array([x, y, 0.])
            R = self._Rz(psi)
            p_w = o_s + R_s @ p
            R_w = R_s @ R
            q_w = Rotation.from_matrix(R_w).as_quat() # <--- (x, y, z, w) same as pybullet
            self.add_ring(
                p_w,
                q_w,
                1.,
                0.25,
                'ring.urdf',
            )
            self.num_rings += 1
        
        # Add rings after arc
        o_s = p2 + a * v23
        R_s = R_s @ self._Rz(max_psi)
        for i in range(1, extra_n + 1):
            x = extra_d_y * i
            y = 0.
            p = np.array([x, y, 0.])
            R = np.eye(3)
            p_w = o_s + R_s @ p
            R_w = R_s @ R
            q_w = Rotation.from_matrix(R_w).as_quat() # <--- (x, y, z, w) same as pybullet
            self.add_ring(
                p_w,
                q_w,
                0.75,
                0.25,
                'ring.urdf',
            )
            self.num_rings += 1

    def add_ring(self, pos, ori, radius, width, urdf):
        if len(ori) == 3:
            # ori is rpy
            q = self.bullet_client.getQuaternionFromEuler(ori)
        elif len(ori) == 4:
            # ori is q
            q = ori
        else:
            raise Exception(f'ori must have length 3 or 4: {ori}')
        R = np.reshape(self.bullet_client.getMatrixFromQuaternion(q), (3, 3))
        id = self.bullet_client.loadURDF(str(Path(f'./urdf/{urdf}')),
                        basePosition=pos,
                        baseOrientation=q,
                        useFixedBase=1)
        self.rings.append({
            'id': id,
            'p': np.array(pos),
            'R': R,
            'q': q,
            'radius': radius,
            'width': width,
        })

    def is_inside_ring(self, ring, q):
        # Put q in the ring frame
        q = ring['R'].T @ (q - ring['p'])
        # Check if q is too far from y-z plane of ring frame
        if np.abs(q[0]) > (ring['width'] / 2):
            return False
        # Check of q is close enough to x axis of ring frame
        return (q[1]**2 + q[2]**2 <= ring['radius']**2)

    def disconnect(self):
        self.bullet_client.disconnect()

    def reset(
            self,
            initial_conditions=None,
            rpy_noise=0.01,
            linvel_noise=0.01,
            angvel_noise=0.01,
            pos_meas_noise=0.01,
            yaw_meas_noise=0.001,
            marker_noise=0.01,
        ):

        # Set marker noise
        self.marker_noise=marker_noise

        # Reset time
        self.max_time_steps = 0
        self.time_step = 0
        self.t = 0.

        # Do nothing else if there are no drones
        if len(self.drones) == 0:
            return

        # Create initial conditions if they haven't already been specified
        if initial_conditions is None:
            initial_conditions = {}
            # Try to find a place for each drone in the start ring
            p = self._get_points(len(self.drones), 0.25, 2.5)
            if p is None:
                raise Exception('Placement failed! Try again.')
            for drone, point in zip(self.drones, p.tolist()):
                # Initial state
                ic = {
                    'p_x': point[0] + self.rings[0]['p'][0],
                    'p_y': point[1] + self.rings[0]['p'][1],
                    'p_z': 0.3,
                    'roll': rpy_noise * self.rng.standard_normal(),
                    'pitch': rpy_noise * self.rng.standard_normal(),
                    'yaw': rpy_noise * self.rng.standard_normal(),
                    'v_x': linvel_noise * self.rng.standard_normal(),
                    'v_y': linvel_noise * self.rng.standard_normal(),
                    'v_z': linvel_noise * self.rng.standard_normal(),
                    'w_x': angvel_noise * self.rng.standard_normal(),
                    'w_y': angvel_noise * self.rng.standard_normal(),
                    'w_z': angvel_noise * self.rng.standard_normal(),
                }
                # Initial measurement
                ic['p_x_meas'] = ic['p_x'] + pos_meas_noise * self.rng.standard_normal()
                ic['p_y_meas'] = ic['p_y'] + pos_meas_noise * self.rng.standard_normal()
                ic['p_z_meas'] = ic['p_z'] + pos_meas_noise * self.rng.standard_normal()
                ic['yaw_meas'] = ic['yaw'] + yaw_meas_noise * self.rng.standard_normal()
                initial_conditions[drone['name']] = ic
        else:
            # Check that each drone has a complete initial condition
            keys_needed = [
                'p_x',
                'p_y',
                'p_z',
                'yaw',
                'pitch',
                'roll',
                'v_x',
                'v_y',
                'v_z',
                'w_x',
                'w_y',
                'w_z',
                'p_x_meas',
                'p_y_meas',
                'p_z_meas',
                'yaw_meas',
            ]
            for drone in self.drones:
                if drone['name'] not in initial_conditions.keys():
                    raise Exception(f'If you specify initial conditions, you must specify them for all drones - they are missing for "{drone["name"]}".')
                for key in keys_needed:
                    if key not in initial_conditions[drone['name']].keys():
                        raise Exception(f'If you specify initial conditions, you must specify all of them - "{key}" is missing for "{drone["name"]}".')
        
        # Set the initial state of each drone
        for drone in self.drones:
            # Get initial conditions
            ic = initial_conditions[drone['name']]
            # Position and orientation
            pos = np.array([ic['p_x'], ic['p_y'], ic['p_z']])
            rpy = np.array([ic['roll'], ic['pitch'], ic['yaw']])
            ori = self.bullet_client.getQuaternionFromEuler(rpy)
            self.bullet_client.resetBasePositionAndOrientation(drone['id'], pos, ori)
            # Linear and angular velocity
            linvel = np.array([ic['v_x'], ic['v_y'], ic['v_z']])
            angvel = np.array([ic['w_x'], ic['w_y'], ic['w_z']])
            self.bullet_client.resetBaseVelocity(drone['id'], linearVelocity=linvel, angularVelocity=angvel)
            # Actuator commands
            drone['u'] = np.zeros(4)
            # Index of target ring
            drone['cur_ring'] = 1
            # Data
            drone['data'] = {
                't': [],
                'p_x': [],
                'p_y': [],
                'p_z': [],
                'yaw': [],
                'pitch': [],
                'roll': [],
                'v_x': [],
                'v_y': [],
                'v_z': [],
                'w_x': [],
                'w_y': [],
                'w_z': [],
                'pos_markers': [],
                'pos_ring': [],
                'dir_ring': [],
                'is_last_ring': [],
                'tau_x': [],
                'tau_y': [],
                'tau_z': [],
                'f_z': [],
                'tau_x_cmd': [],
                'tau_y_cmd': [],
                'tau_z_cmd': [],
                'f_z_cmd': [],
                'run_time': [],
            }
            # Finish time
            drone['finish_time'] = None
            # Still running
            drone['running'] = True
            # Number of run time violations
            drone['num_run_time_violations'] = 0
            
            # Initialize controller
            try:
                controller_start_time = time.time()
                if self.error_on_print:
                    with contextlib.redirect_stdout(io.StringIO()) as stdout:
                        drone['controller'].reset(
                            ic['p_x_meas'],
                            ic['p_y_meas'],
                            ic['p_z_meas'],
                            ic['yaw_meas'],
                        )
                    stdout_val = stdout.getvalue()
                    if stdout_val:
                        raise Exception(f'Printed the following text to stdout, which is forbidden:\n\n{stdout_val}')
                else:
                    drone['controller'].reset(
                        ic['p_x_meas'],
                        ic['p_y_meas'],
                        ic['p_z_meas'],
                        ic['yaw_meas'],
                    )
                controller_run_time = time.time() - controller_start_time
                if (controller_run_time > self.max_controller_reset_time) and self.error_on_timeout:
                    raise Exception(f'Reset timeout exceeded: {controller_run_time} > {self.max_controller_reset_time}')
                
                # Try to add user-defined variables to data log
                for key in drone['variables_to_log']:
                    if key in drone['data'].keys():
                        raise Exception(f'Trying to log duplicate variable {key} (choose a different name)')
                    drone['data'][key] = []
            except Exception as err:
                print(f'\n==========\nerror on reset of drone {drone["name"]} (turning it off):\n==========\n{traceback.format_exc()}==========\n')
                drone['running'] = False
                continue

        # Update camera and display
        self._update_camera()
        self._update_display()

    def enforce_motor_limits(self, tau_x_des, tau_y_des, tau_z_des, f_z_des):
        # pack inputs into array
        u = np.array([tau_x_des, tau_y_des, tau_z_des, f_z_des])
        # compute and bound squared spin rates
        s = np.clip(self.M @ u, self.s_min, self.s_max)
        # recompute inputs
        u = linalg.solve(self.M, s)
        return u[0], u[1], u[2], u[3]

    def set_actuator_commands(self, tau_x_des, tau_y_des, tau_z_des, f_z_des, drone):
        tau_x, tau_y, tau_z, f_z = self.enforce_motor_limits(tau_x_des, tau_y_des, tau_z_des, f_z_des)
        drone['u'] = np.array([tau_x, tau_y, tau_z, f_z])
        return tau_x, tau_y, tau_z, f_z

    def get_sensor_measurements(self, drone):
        # Get marker positions
        pos, ori = self.bullet_client.getBasePositionAndOrientation(drone['id'])
        p_body_in_world = np.array(pos)
        R_body_in_world = np.reshape(np.array(self.bullet_client.getMatrixFromQuaternion(ori)), (3, 3))
        a_in_body = np.array([self.l, 0., 0.])
        b_in_body = np.array([-self.l, 0., 0.])
        a_in_world = p_body_in_world + R_body_in_world @ a_in_body
        b_in_world = p_body_in_world + R_body_in_world @ b_in_body

        # Get measurements of marker positions
        pos_markers = np.concatenate([a_in_world, b_in_world]) + self.marker_noise * self.rng.standard_normal(6)
        
        # Get information about next ring
        pos_ring = self.rings[drone['cur_ring']]['p'].copy()
        dir_ring = self.rings[drone['cur_ring']]['R'][:, 0].copy()
        is_last_ring = ((drone['cur_ring'] + 1) == len(self.rings))

        return pos_markers, pos_ring, dir_ring, is_last_ring

    def get_state(self, drone):
        pos, ori = self.bullet_client.getBasePositionAndOrientation(drone['id'])
        rpy = self.bullet_client.getEulerFromQuaternion(ori)
        vel = self.bullet_client.getBaseVelocity(drone['id'])
        v_world = np.array(vel[0])
        w_world = np.array(vel[1])
        R_body_in_world = np.reshape(np.array(self.bullet_client.getMatrixFromQuaternion(ori)), (3, 3))
        v_body = R_body_in_world.T @ v_world
        w_body = R_body_in_world.T @ w_world
        return np.array(pos), np.array(rpy), v_body, w_body

    def check_ring(self, drone):
        pos, ori = self.bullet_client.getBasePositionAndOrientation(drone['id'])
        pos = np.array(pos)
        if self.is_inside_ring(self.rings[drone['cur_ring']], pos):
            drone['cur_ring'] += 1
        if drone['cur_ring'] == len(self.rings):
            drone['finish_time'] = self.t
            return True
        else:
            return False

    def run(
            self,
            max_time=None,
            video_filename=None,
            print_debug=False
        ):

        if max_time is None:
            self.max_time_steps = None
        else:
            self.max_time_steps = 1 + int((max_time + self.t) / self.dt)
        self.start_time = time.time() - self.t
        start_time_step = self.time_step

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
            all_done = self.step(print_debug=print_debug)

            if self.display_meshcat or self.display_pybullet:
                self._update_camera()
            
            if self.display_meshcat:
                self.meshcat_update()

            if video_filename is not None:
                if self.time_step % 25 == 0:
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
        
        stop_time = time.time()
        stop_time_step = self.time_step

        elapsed_time = stop_time - self.start_time
        elapsed_time_steps = stop_time_step - start_time_step
        if (elapsed_time > 0) and print_debug:
            print(f'Simulated {elapsed_time_steps} time steps in {elapsed_time:.4f} seconds ({(elapsed_time_steps / elapsed_time):.4f} time steps per second)')
    

    def get_data(self, drone_name):
        # Try to get drone by name, returning if none exists
        drone = self.get_drone_by_name(drone_name)
        if drone is None:
            drone_names = '\n'.join([d['name'] for d in self.drones])
            msg = f'The simulator has no drone with name "{drone_name}".'
            if len(drone_names) == 0:
                msg += f' The simulator has no drones at all, in fact.'
            else:
                msg += f' The simulator has these drones:'
                msg += f'\n==========\n{drone_names}\n==========\n'
            print(msg)
            return None
        
        # Convert lists to numpy arrays and return the result
        data = drone['data'].copy()
        for key in data.keys():
            data[key] = np.array(data[key])
        return data
    

    def get_result(self, drone_name):
        # Try to get drone by name, returning if none exists
        drone = self.get_drone_by_name(drone_name)
        if drone is None:
            drone_names = '\n'.join([d['name'] for d in self.drones])
            msg = f'The simulator has no drone with name "{drone_name}".'
            if len(drone_names) == 0:
                msg += f' The simulator has no drones at all, in fact.'
            else:
                msg += f' The simulator has these drones:'
                msg += f'\n==========\n{drone_names}\n==========\n'
            print(msg)
            return None
        
        # Get and return the result (did it fail, did it finish, when did it finish)
        if drone['finish_time'] is None:
            if drone['running']:
                failed = False
                finished = False
                finish_time = None
            else:
                failed = True
                finished = False
                finish_time = None
        else:
            failed = False
            finished = True
            finish_time = drone['finish_time']
        return failed, finished, finish_time

    def step(self, print_debug=False):
        """
        does one step in the simulation
        """

        # current time
        self.t = self.time_step * self.dt

        # get position of all drones
        all_pos = []
        for drone in self.drones:
            pos, ori = self.bullet_client.getBasePositionAndOrientation(drone['id'])
            all_pos.append(pos)
        all_pos = np.array(all_pos)

        all_done = True
        for index, drone in enumerate(self.drones):
            # ignore the drone if it is not still running
            if not drone['running']:
                continue

            # check if the drone has just now finished, and if so ignore it
            if self.check_ring(drone):
                if print_debug:
                    print(f'FINISHED: drone "{drone["name"]}" at time {drone["finish_time"]:.2f}')
                drone['running'] = False
                continue

            # the drone is not finished, so the simulation should continue
            all_done = False

            # get state
            pos, rpy, linvel, angvel = self.get_state(drone)

            # get measurements
            pos_markers, pos_ring, dir_ring, is_last_ring = self.get_sensor_measurements(drone)

            # get actuator commands
            try:
                controller_start_time = time.time()
                if self.error_on_print:
                    with contextlib.redirect_stdout(io.StringIO()) as stdout:
                        (
                            tau_x_cmd,
                            tau_y_cmd,
                            tau_z_cmd,
                            f_z_cmd,
                        ) = drone['controller'].run(
                            pos_markers,
                            pos_ring,
                            dir_ring,
                            is_last_ring,
                            np.delete(all_pos, index, axis=0),
                        )
                    stdout_val = stdout.getvalue()
                    if stdout_val:
                        raise Exception(f'Printed the following text to stdout, which is forbidden:\n\n{stdout_val}')
                else:
                    (
                        tau_x_cmd,
                        tau_y_cmd,
                        tau_z_cmd,
                        f_z_cmd,
                    ) = drone['controller'].run(
                        pos_markers,
                        pos_ring,
                        dir_ring,
                        is_last_ring,
                        np.delete(all_pos, index, axis=0),
                    )
                controller_run_time = time.time() - controller_start_time
                if (controller_run_time > self.max_controller_run_time):
                    drone['num_run_time_violations'] += 1
                if (drone['num_run_time_violations'] >= self.max_run_time_violations) and self.error_on_timeout:
                    raise Exception(f'Maximum run time of {self.max_controller_run_time} was exceeded on {self.max_run_time_violations} occasions')

                (
                    tau_x,
                    tau_y,
                    tau_z,
                    f_z,
                ) = self.set_actuator_commands(
                    tau_x_cmd,
                    tau_y_cmd,
                    tau_z_cmd,
                    f_z_cmd,
                    drone,
                )
            except Exception as err:
                if print_debug:
                    print(f'\n==========\nerror on run of drone {drone["name"]} (turning it off):\n==========\n{traceback.format_exc()}==========\n')
                drone['running'] = False
                continue

            # apply rotor forces
            self.bullet_client.applyExternalForce(
                drone['id'],
                drone['link_map']['center_of_mass'],
                np.array([0., 0., drone['u'][3]]),
                np.array([0., 0., 0.]),
                self.bullet_client.LINK_FRAME,
            )

            # apply rotor torques
            self.bullet_client.applyExternalTorque(
                drone['id'],
                drone['link_map']['center_of_mass'],
                np.array([drone['u'][0], drone['u'][1], drone['u'][2]]),
                self.bullet_client.LINK_FRAME,
            )

            # log data
            data = drone['data']
            data['t'].append(self.t)
            data['p_x'].append(pos[0])
            data['p_y'].append(pos[1])
            data['p_z'].append(pos[2])
            data['yaw'].append(rpy[2])
            data['pitch'].append(rpy[1])
            data['roll'].append(rpy[0])
            data['v_x'].append(linvel[0])
            data['v_y'].append(linvel[1])
            data['v_z'].append(linvel[2])
            data['w_x'].append(angvel[0])
            data['w_y'].append(angvel[1])
            data['w_z'].append(angvel[2])
            data['pos_markers'].append(pos_markers)
            data['pos_ring'].append(pos_ring)
            data['dir_ring'].append(dir_ring)
            data['is_last_ring'].append(is_last_ring)
            data['tau_x'].append(tau_x)
            data['tau_y'].append(tau_y)
            data['tau_z'].append(tau_z)
            data['f_z'].append(f_z)
            data['tau_x_cmd'].append(tau_x_cmd)
            data['tau_y_cmd'].append(tau_y_cmd)
            data['tau_z_cmd'].append(tau_z_cmd)
            data['f_z_cmd'].append(f_z_cmd)
            data['run_time'].append(controller_run_time)
            try:
                for key in drone['variables_to_log']:
                    val = getattr(drone['controller'], key, np.nan)
                    if not np.isscalar(val):
                        val = val.flatten().tolist()
                    data[key].append(val)
            except Exception as err:
                print(f'\n==========\nerror logging data for drone {drone["name"]} (turning it off):\n==========\n{traceback.format_exc()}==========\n')
                drone['running'] = False
                continue
            
            # check for inactivity
            if self.error_on_inactive:
                if len(data['t']) > self.activity_n:
                    dx = np.max(data['p_x'][-self.activity_n:]) - np.min(data['p_x'][-self.activity_n:])
                    dy = np.max(data['p_y'][-self.activity_n:]) - np.min(data['p_y'][-self.activity_n:])
                    dz = np.max(data['p_z'][-self.activity_n:]) - np.min(data['p_z'][-self.activity_n:])
                    d = np.max([dx, dy, dz])
                    if d < self.activity_d:
                        if print_debug:
                            print(f'\n==========\ndrone {drone["name"]} is inactive (turning it off)\n==========\n')
                        drone['running'] = False
                        continue
            
            # check for out-of-bounds
            if self._drone_is_out_of_bounds(pos):
                if print_debug:
                    print(f'\n==========\ndrone {drone["name"]} is out of bounds (turning it off)\n==========\n')
                drone['running'] = False
                continue


        # try to stay real-time
        if self.display_pybullet or self.display_meshcat:
            t = self.start_time + (self.dt * (self.time_step + 1))
            time_to_wait = t - time.time()
            while time_to_wait > 0:
                time.sleep(0.9 * time_to_wait)
                time_to_wait = t - time.time()

        # take a simulation step
        self.bullet_client.stepSimulation()

        # increment time step
        self.time_step += 1

        return all_done
    
    def _drone_is_out_of_bounds(self, pos):
        if pos[0] <= -self.bounds_d:
            return True
        if pos[0] >= self.bounds_d:
            return True
        if pos[1] <= -self.bounds_d:
            return True
        if pos[1] >= self.bounds_d:
            return True
        # (We assume that p_z is never less than zero,
        #  because of the ground.)
        if pos[2] >= self.bounds_d:
            return True
        return False

    def get_drone_by_name(self, name):
        for drone in self.drones:
            if drone['name'] == name:
                return drone
        return None

    def show_results(self):
        finished = []
        still_running = []
        failed = []
        for drone in self.drones:
            if drone['finish_time'] is not None:
                finished.append((drone['name'], drone['finish_time']))
            elif drone['running']:
                still_running.append(drone['name'])
            else:
                failed.append(drone['name'])

        sorted(finished, key=lambda f: f[1])
        print('FINISHED')
        for d in finished:
            print(f' {d[0]:20s} : {d[1]:6.2f}')

        print('\nSTILL RUNNING')
        for d in still_running:
            print(f' {d:20s}')

        print('\nFAILED')
        for d in failed:
            print(f' {d:20s}')

    def pybullet_snapshot(self):
        # Note: you *must* specify a projectionMatrix when calling getCameraImage,
        # or you will get whatever view is currently shown in the GUI.

        # World view (always from finish)
        p = np.array(self.rings[-1]['p'])
        p_eye = np.array([p[0] + 5., p[1], 2.])
        p_target = np.array([p[0], p[1], 1.])
        v_up = np.array([0., 0., 1.])
        view_matrix = self.bullet_client.computeViewMatrix(p_eye, p_target, v_up)
        projection_matrix = self.bullet_client.computeProjectionMatrixFOV(fov=120, aspect=1.0, nearVal=0.01, farVal=100.0)
        im = self.bullet_client.getCameraImage(480, 480, viewMatrix=view_matrix, projectionMatrix=projection_matrix, renderer=self.bullet_client.ER_BULLET_HARDWARE_OPENGL, shadow=1)
        rgba_world = im[2]

        # Body view (picture-in-picture)
        if (self.view['type'] == 'drone') and (self.view['drone_name']) is not None:
            drone = self.get_drone_by_name(self.view['drone_name'])
            if drone is None:
                raise Exception(f'drone "{drone_name}" does not exist')
            pos, ori = self.bullet_client.getBasePositionAndOrientation(drone['id'])
            o_body_in_world = np.array(pos)
            R_body_in_world = np.reshape(np.array(self.bullet_client.getMatrixFromQuaternion(ori)), (3, 3))
            p_eye = o_body_in_world + R_body_in_world @ np.array([-1.5, 0., 0.5])
            p_target = o_body_in_world + R_body_in_world @ np.array([0.5, 0., 0.])
            v_up = (R_body_in_world[:, 2]).flatten()
            view_matrix = self.bullet_client.computeViewMatrix(p_eye, p_target, v_up)
            projection_matrix = self.bullet_client.computeProjectionMatrixFOV(fov=60.0, aspect=1.0, nearVal=0.01, farVal=100.0)
            im = self.bullet_client.getCameraImage(128, 128, viewMatrix=view_matrix, projectionMatrix=projection_matrix, renderer=self.bullet_client.ER_BULLET_HARDWARE_OPENGL, shadow=0)
            rgba_body = im[2]
            rgba_world[10:138, 10:138, :] = rgba_body

        return rgba_world
    
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

    def _get_rep_grad(self, p, i, params):
        gradfrep = np.zeros(2)
        for j in range(p.shape[0]):
            if j != i:
                v = p[i] - p[j]
                vnorm = np.linalg.norm(v)
                d = vnorm
                dgrad = v / vnorm
                if (d <= params['brep']):
                    gradfrep += params['krep'] * ((1 / params['brep']) - (1 / d)) * (1 / (d ** 2)) * dgrad
        v = p[i]
        vnorm = np.linalg.norm(v)
        d = params['radius'] - vnorm
        dgrad = - v / vnorm
        if (d <= params['brep']):
            gradfrep += params['krep'] * ((1 / params['brep']) - (1 / d)) * (1 / (d ** 2)) * dgrad
        d = np.linalg.norm(gradfrep)
        if (d >= params['max_step']):
            gradfrep *= (params['max_step'] / d)
        return gradfrep

    def _get_step(self, p, params):
        dp = []
        for i in range(p.shape[0]):
            dp.append(- params['kdes'] * self._get_rep_grad(p, i, params))
        return np.array(dp)

    def _get_dmin_for_point(self, p, j, params):
        dmin = params['radius'] - np.linalg.norm(p[j])
        for i in range(p.shape[0]):
            if i != j:
                d = np.linalg.norm(p[i] - p[j])
                if d < dmin:
                    dmin = d
        return dmin

    def _get_dmin(self, p, params):
        dmin = np.inf
        for i in range(p.shape[0]):
            d = self._get_dmin_for_point(p, i, params)
            if d < dmin:
                dmin = d
        return dmin

    def _get_points(self, num_points, inner_radius, outer_radius):
        # sample points in circle
        pr = self.rng.uniform(low=0., high=outer_radius, size=(num_points,))
        ph = self.rng.uniform(low=0., high=2*np.pi, size=(num_points,))
        p = (pr * np.array([np.cos(ph), np.sin(ph)])).T
        # do 25 steps of gradient descent to spread out the points
        params = {
            'krep': 1.,
            'brep': 4 * inner_radius,
            'katt': 1.,
            'batt': 1.,
            'kdes': 5e-1,
            'radius': outer_radius,
            'max_step': 0.1,
        }
        for i in range(50):
            p += self._get_step(p, params)
        if self._get_dmin(p, params) > 2 * inner_radius:
            return p
        else:
            return None

    def _update_display(self):
        if self.display_pybullet:
            # hack to get GUI to update on MacOS
            time.sleep(0.01)
            keys = self.bullet_client.getKeyboardEvents()
        
        if self.display_meshcat:
            self.meshcat_update()

    def _smooth_view(self, camera_target, camera_roll, camera_pitch, camera_yaw, m=0.02):
        self.camera_target = ((1 - m) * np.array(self.view['prev_camera_target']) + m * np.array(camera_target)).tolist()
        self.view['prev_camera_target'] = self.camera_target
        self.camera_roll = ((1 - m) * self.view['prev_camera_roll']) + m * camera_roll
        self.view['prev_camera_roll'] = self.camera_roll
        self.camera_pitch = ((1 - m) * self.view['prev_camera_pitch']) + m * camera_pitch
        self.view['prev_camera_pitch'] = self.camera_pitch
        self.camera_yaw = ((1 - m) * self.view['prev_camera_yaw']) + m * camera_yaw
        self.view['prev_camera_yaw'] = self.camera_yaw

        if self.display_pybullet:
            self.bullet_client.resetDebugVisualizerCamera(
                self.camera_distance,
                self.camera_yaw - 90,
                -self.camera_pitch,
                self.camera_target,
            )

    def _update_camera(self):
        if not (self.display_meshcat or self.display_pybullet):
            return
        
        # Drone view
        if self.view['type'] == 'drone':
            drone_name = self.view['drone_name']
            drone = self.get_drone_by_name(drone_name)
            if drone is None:
                raise Exception(f'drone "{drone_name}" does not exist')
            pos, ori = self.bullet_client.getBasePositionAndOrientation(drone['id'])
            eul = self.bullet_client.getEulerFromQuaternion(ori)

            self.camera_roll = 0.
            self.camera_pitch = 15.
            self.camera_yaw = ((eul[2] * 180 / np.pi) + (self.view['yaw'] - 270))
            self.camera_target = pos

            if self.display_pybullet:
                self.bullet_client.resetDebugVisualizerCamera(
                    self.camera_distance,
                    self.camera_yaw - 90,
                    -self.camera_pitch,
                    self.camera_target,
                )
            
            return
        
        # Contest view
        if self.view['type'] == 'contest':
            if len(self.drones) == 0:
                raise Exception('there must be at least one drone to show contest view')

            # If any drone is finished, then stay at finish view (smoothed)
            for drone in self.drones:
                if drone['finish_time'] is not None:
                    p = self.rings[-1]['p']
                    self._smooth_view(
                        [p[0], p[1], 1.],
                        0.,
                        0.,
                        180.,
                    )
                    return
            
            # Find the farthest ring toward which any drone is (or was) heading
            max_ring_index = 0
            for drone in self.drones:
                if drone['cur_ring'] > max_ring_index:
                    max_ring_index = drone['cur_ring']
            
            # Find the projected distance between this ring and the previous ring
            ring_a = self.rings[max_ring_index - 1]
            ring_b = self.rings[max_ring_index]
            q = ring_b['R'].T @ (ring_a['p'] - ring_b['p'])
            max_dist = - q[0]

            # Find the minimum projected distance between this ring and the position
            # of any drone heading toward it
            min_dist = max_dist
            for drone in self.drones:
                if drone['cur_ring'] == max_ring_index:
                    pos, ori = self.bullet_client.getBasePositionAndOrientation(drone['id'])
                    q = ring_b['R'].T @ (pos - ring_b['p'])
                    d = - q[0]
                    if d < min_dist:
                        min_dist = d
            
            # It is possible for this minimum projected distance to be negative (e.g., 
            # if a drone went beyond the ring but not through it). In this case, cap it
            # at zero.
            min_dist = max(min_dist, 0.)
            
            # Make sure this minimum projected distance is non-infinite
            if min_dist == np.inf:
                raise Exception('min_dist is infinite (something very bad happened)')
            
            # Find camera target, interpolated between previous ring and current ring
            m = (max_dist - min_dist) / max_dist
            camera_target = (1 - m) * ring_a['p'] + m * ring_b['p']

            # Smooth the view
            self._smooth_view(
                camera_target,
                0.,
                0.,
                180.,
            )

            return
        
    def meshcat_camera(self):
        # By default, the meshcat camera points in the -x direction.
        # So, by rotating this camera 180 degrees about the +z axis,
        # we make it point in the +x direction (with +y up).
        self.vis['/Cameras/default'].set_transform(
            meshcat.transformations.rotation_matrix(np.pi, [0., 0., 1.])
        )
        self.vis['/Cameras/default/rotated/<object>'].set_property(
            'position', [self.camera_distance, 0., 0.],
        )
        self.vis['/Cameras/default/rotated/<object>'].set_property(
            'fov', 120,
        )

    def camera_startview(self):
        self.view = {
            'type': 'start',
        }

        self.camera_target = self.rings[0]['p'] + np.array([0., 0., 1.])
        self.camera_yaw = 0.
        self.camera_pitch = 0.
        self.camera_roll = 0.
        self.camera_distance = 5.
        
        if not (self.display_meshcat or self.display_pybullet):
            return

        if self.display_pybullet:
            # self.bullet_client.resetDebugVisualizerCamera(5, -90, -30, [0., 0., 0.])
            self.bullet_client.resetDebugVisualizerCamera(
                self.camera_distance,
                self.camera_yaw - 90,
                -self.camera_pitch,
                self.camera_target,
            )
        
        if self.display_meshcat:
            self.meshcat_camera()
        
        self._update_display()

    def camera_finishview(self):
        self.view = {
            'type': 'finish',
        }

        self.camera_roll = 0.
        self.camera_pitch = 30.
        self.camera_yaw = 180.
        self.camera_distance = 5.
        self.camera_target = self.rings[-1]['p']

        if not (self.display_meshcat or self.display_pybullet):
            return

        if self.display_pybullet:
            # self.bullet_client.resetDebugVisualizerCamera(5, 90, -30, self.rings[-1]['p'])
            self.bullet_client.resetDebugVisualizerCamera(
                self.camera_distance,
                self.camera_yaw - 90,
                -self.camera_pitch,
                self.camera_target,
            )
        
        if self.display_meshcat:
            self.meshcat_camera()
        
        self._update_display()
    
    def camera_contestview(self):
        self.view = {
            'type': 'contest',
            'prev_camera_target': self.camera_target,
            'prev_camera_roll': self.camera_roll,
            'prev_camera_pitch': self.camera_pitch,
            'prev_camera_yaw': self.camera_yaw,
        }

        self.camera_distance = 3.

        p = self.rings[-1]['p']
        self.camera_target = [p[0], p[1], 1.]
        self.camera_yaw = 0.
        self.camera_pitch = 0.
        self.camera_roll = 0.

        if self.display_meshcat:
            self.meshcat_camera()
        
        self._update_camera()
        self._update_display()

    def camera_droneview(self, drone_name, yaw=270.):
        """
        view from right side: yaw=0
        view from front: yaw=90
        view from left side: yaw=180
        view from back: yaw=270
        """

        if drone_name is None:
            raise Exception('must specify drone_name')
        
        if self.get_drone_by_name(drone_name) is None:
            raise Exception(f'drone "{drone_name}" does not exist')

        self.view = {
            'type': 'drone',
            'drone_name': drone_name,
            'yaw': yaw,
        }
        
        self.camera_distance = 3.
        
        if self.display_meshcat:
            self.meshcat_camera()
        
        self._update_camera()
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

    def meshcat_add_drone(self, drone):
        if not self.display_meshcat:
            return
        
        self.vis['scene']['drones'][drone['name']].set_transform(meshcat.transformations.identity_matrix())
        
        links = []
        for s in self.bullet_client.getVisualShapeData(drone['id']):
            link_id = s[1]
            stl_filename = s[4].decode('UTF-8')
            scale = s[3]
            color = self._convert_color(s[7])

            # It is only possible to ask pybullet for the name of non-base links
            is_base = (link_id == -1)
            if is_base:
                link_name = 'base'
            else:
                joint_info = self.bullet_client.getJointInfo(drone['id'], link_id)
                link_name = joint_info[12].decode('UTF-8')
            
            links.append({
                'name': link_name,
                'id': link_id,
                'scale': scale,
                'is_base': is_base,
            })

            if link_name == 'screen':
                self.vis['scene']['drones'][drone['name']][link_name].set_object(
                    meshcat.geometry.ObjMeshGeometry.from_file(stl_filename),
                    meshcat.geometry.MeshPhongMaterial(
                        color=color['color'],
                        transparent=color['transparent'],
                        opacity=color['opacity'],
                        reflectivity=0.8,
                        map=meshcat.geometry.ImageTexture(
                            image=meshcat.geometry.PngImage.from_file(str(Path(drone['image']))),
                            wrap=[1, 1],
                            repeat=[1, 1],
                        ),
                    )
                )
            else:
                self.vis['scene']['drones'][drone['name']][link_name].set_object(
                    meshcat.geometry.StlMeshGeometry.from_file(stl_filename),
                    meshcat.geometry.MeshPhongMaterial(
                        color=color['color'],
                        transparent=color['transparent'],
                        opacity=color['opacity'],
                        reflectivity=0.8,
                    )
                )
            
            # - Transform
            pos = s[5]
            ori = s[6]
            T = meshcat.transformations.quaternion_matrix(self._wxyz_from_xyzw(ori))
            T[:3, 3] = np.array(pos)[:3]
            S = np.diag(np.concatenate((scale, [1.0])))
            self.vis['scene']['drones'][drone['name']][link_name].set_transform(T @ S)

        drone['links'] = links

    def meshcat_clear_drones(self):
        if not self.display_meshcat:
            return
        
        self.vis['scene']['drones'].delete()
    
    def meshcat_add_rings(self):
        if not self.display_meshcat:
            return

        for ring in self.rings:
            s = self.bullet_client.getVisualShapeData(ring['id'])[0]
            if s[1] != -1:
                raise Exception('bad ring link id')
            stl_filename = s[4].decode('UTF-8')
            scale = s[3]
            color = self._convert_color(s[7])
            ring['name'] = f'{ring["id"]}'
            ring['scale'] = scale
            # - Object
            self.vis['scene']['rings'][ring['name']].set_object(
                meshcat.geometry.StlMeshGeometry.from_file(stl_filename),
                meshcat.geometry.MeshPhongMaterial(
                    color=color['color'],
                    transparent=color['transparent'],
                    opacity=color['opacity'],
                    reflectivity=0.8,
                )
            )
            # - Transform
            pos, ori = self.bullet_client.getBasePositionAndOrientation(ring['id'])
            S = np.diag(np.concatenate((ring['scale'], [1.0])))
            T = meshcat.transformations.quaternion_matrix(self._wxyz_from_xyzw(ori))
            T[:3, 3] = np.array(pos)[:3]
            self.vis['scene']['rings'][ring['name']].set_transform(T @ S)

    def meshcat_clear_rings(self):
        if not self.display_meshcat:
            return
        
        self.vis['scene']['rings'].delete()

    def meshcat_init(self):
        # Create a visualizer
        self.vis = meshcat.Visualizer().open()

        # Make sure everything has been deleted from the visualizer
        self.vis.delete()

        # Base for everything 
        self.vis['scene'].set_transform(meshcat.transformations.identity_matrix())

        # Add plane
        # - Object
        self.vis['scene']['plane'].set_object(
            meshcat.geometry.ObjMeshGeometry.from_file(str(Path('./urdf/plane.obj'))),
            meshcat.geometry.MeshPhongMaterial(
                map=meshcat.geometry.ImageTexture(
                    image=meshcat.geometry.PngImage.from_file(str(Path('./urdf/checker_blue.png'))),
                    wrap=[1, 1],
                    repeat=[1, 1],
                ),
            ),
        )
        # - Transform
        self.plane_scale = self.bullet_client.getVisualShapeData(self.plane_id)[0][3]
        pos_and_ori = self.bullet_client.getBasePositionAndOrientation(self.plane_id)
        pos = pos_and_ori[0]
        ori = pos_and_ori[1]
        S = np.diag(np.concatenate((self.plane_scale, [1.0])))
        T = meshcat.transformations.quaternion_matrix(self._wxyz_from_xyzw(ori))
        T[:3, 3] = np.array(pos)[:3]
        self.vis['scene']['plane'].set_transform(T @ S)

        # Turn off grid
        self.vis['/Grid'].set_property('visible', False)

        # Set background color
        self.vis['/Background'].set_property('top_color', [0, 0, 1])
        self.vis['/Background'].set_property('bottom_color', [0.95, 0.80, 0.60])
        
    def meshcat_update(self):

        #
        # TODO   Use getLinkStates instead of getLinkState to make
        #        this function more efficient.
        #

        # Move scene to move camera, because meshcat-python does not
        # fully support camera control
        T_translate = meshcat.transformations.translation_matrix(self.camera_target)
        T_rotate = meshcat.transformations.euler_matrix(
            np.deg2rad(self.camera_yaw),
            np.deg2rad(self.camera_pitch),
            np.deg2rad(self.camera_roll),
            'rzyx',
        )
        T_cam_in_world = T_translate @ T_rotate
        T_world_in_cam = meshcat.transformations.inverse_matrix(T_cam_in_world)

        # Scene
        self.vis['scene'].set_transform(T_world_in_cam)

        # Drones (relative to scene)
        for drone in self.drones:
            pos, ori = self.bullet_client.getBasePositionAndOrientation(drone['id'])
            T = meshcat.transformations.quaternion_matrix(self._wxyz_from_xyzw(ori))
            T[:3, 3] = np.array(pos)[:3]
            self.vis['scene']['drones'][drone['name']].set_transform(T)