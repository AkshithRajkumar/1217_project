"""Write your proposed algorithm.
[NOTE]: The idea for the final project is to plan the trajectory based on a sequence of gates 
while considering the uncertainty of the obstacles. The students should show that the proposed 
algorithm is able to safely navigate a quadrotor to complete the task in both simulation and
real-world experiments.

Then run:

    $ python3 final_project.py --overrides ./getting_started.yaml

Tips:
    Search for strings `INSTRUCTIONS` and `REPLACE THIS (START)` in this file.

    Change the code between the 5 blocks starting with
        #########################
        # REPLACE THIS (START) ##
        #########################
    and ending with
        #########################
        # REPLACE THIS (END) ####
        #########################
    with your own code.

    They are in methods:
        1) planning
        2) cmdFirmware

"""
import numpy as np
import math
from collections import deque

try:
    from project_utils import Command, PIDController, timing_step, timing_ep, plot_trajectory, draw_trajectory
except ImportError:
    # PyTest import.
    from .project_utils import Command, PIDController, timing_step, timing_ep, plot_trajectory, draw_trajectory
GlobalR = 0.46
#########################
# REPLACE THIS (START) ##
#########################

# Optionally, create and import modules you wrote.
# Please refrain from importing large or unstable 3rd party packages.

from gate import Gate
import random
import heapq
from scipy.interpolate import CubicSpline
try:
    import example_custom_utils as ecu
except ImportError:
    # PyTest import.
    from . import example_custom_utils as ecu

#########################
# REPLACE THIS (END) ####
#########################

class Controller():
    """Template controller class.

    """

    def __init__(self,
                 initial_obs,
                 initial_info,
                 use_firmware: bool = False,
                 buffer_size: int = 100,
                 verbose: bool = False
                 ):
        """Initialization of the controller.

        INSTRUCTIONS:
            The controller's constructor has access the initial state `initial_obs` and the a priori infromation
            contained in dictionary `initial_info`. Use this method to initialize constants, counters, pre-plan
            trajectories, etc.

        Args:
            initial_obs (ndarray): The initial observation of the quadrotor's state
                [x, x_dot, y, y_dot, z, z_dot, phi, theta, psi, p, q, r].
            initial_info (dict): The a priori information as a dictionary with keys
                'symbolic_model', 'nominal_physical_parameters', 'nominal_gates_pos_and_type', etc.
            use_firmware (bool, optional): Choice between the on-board controll in `pycffirmware`
                or simplified software-only alternative.
            buffer_size (int, optional): Size of the data buffers used in method `learn()`.
            verbose (bool, optional): Turn on and off additional printouts and plots.

        """
        # Save environment and control parameters.
        self.CTRL_TIMESTEP = initial_info["ctrl_timestep"]
        self.CTRL_FREQ = initial_info["ctrl_freq"]
        self.initial_obs = initial_obs
        self.VERBOSE = verbose
        self.BUFFER_SIZE = buffer_size

        # Store a priori scenario information.
        # plan the trajectory based on the information of the (1) gates and (2) obstacles. 
        self.NOMINAL_GATES = initial_info["nominal_gates_pos_and_type"]
        self.NOMINAL_OBSTACLES = initial_info["nominal_obstacles_pos"]

        # Check for pycffirmware.
        if use_firmware:
            self.ctrl = None
        else:
            # Initialize a simple PID Controller for debugging and test.
            # Do NOT use for the IROS 2022 competition. 
            self.ctrl = PIDController()
            # Save additonal environment parameters.
            self.KF = initial_info["quadrotor_kf"]

        # Reset counters and buffers.
        self.reset()
        self.interEpisodeReset()

        # perform trajectory planning
        t_scaled = self.planning(use_firmware, initial_info)

        ## visualization
        # Plot trajectory in each dimension and 3D.
        plot_trajectory(t_scaled, self.waypoints, self.ref_x, self.ref_y, self.ref_z)

        # Draw the trajectory on PyBullet's GUI.
        draw_trajectory(initial_info, self.waypoints, self.ref_x, self.ref_y, self.ref_z)
    
    def obstacle_list(self, NOMINAL_OBSTACLES, NOMINAL_GATES) :
        ObstacleList = []
        for i in self.NOMINAL_OBSTACLES :
            ObstacleList.append([i[0], i[1], i[2], i[5],0])
        for j in self.NOMINAL_GATES :
            ObstacleList.append([j[0], j[1], j[2], j[5],1])
        return ObstacleList
    
    def remove_goal_gate(self, ObstacleList, destination):
        for i in range(len(ObstacleList)) :
            if ObstacleList[i][4] == 1 :
                if ObstacleList[i][0] == destination[0] and ObstacleList[i][1] == destination[1] :
                    ObstacleList.pop(i)
                    break
        return ObstacleList


    def check_obstacle(self, PointX, PointY, ObstacleX, ObstacleY, R) :
        Dist = math.sqrt((PointX - ObstacleX)**2 + (PointY - ObstacleY)**2)
        if Dist > R:
            return False
        return True
    
    def points_on_circumference(self, center, radius, num_points):
        points = []
        cx, cy = center  # Center coordinates
        for i in range(num_points):
            angle = (2 * math.pi * i) / num_points
            x = cx + radius * math.cos(angle)
            y = cy + radius * math.sin(angle)
            points.append((x, y, 1))
        return points
    
    def create_peripheral_coordinates_around_obstcl(self, ObstacleX, ObstacleY, R) :
        return self.points_on_circumference((ObstacleX, ObstacleY), R, 50)

    
    # def remove_obstacle(self, PointX, PointY, ObstacleX, ObstacleY, R, prevCoorX, prevCoorY) :
    #     Point1 = [PointX + R, PointY]
    #     Point2 = [PointX - R, PointY]
    #     Point3 = [PointX, PointY + R]
    #     Point4 = [PointX, PointY - R]

    #     Dist1 = math.sqrt((Point1[0] - prevCoorX)**2 + (Point1[1] - prevCoorY)**2)
    #     Dist2 = math.sqrt((Point3[0] - prevCoorX)**2 + (Point3[1] - prevCoorY)**2)

    #     if Dist1 < Dist2 :
    #         if self.check_obstacle(Point1[0], Point1[1], ObstacleX, ObstacleY, R) :
    #             return Point1
    #         else :
    #             return Point2
    #     else :
    #         if self.check_obstacle(Point3[0], Point3[1], ObstacleX, ObstacleY, R) :
    #             return Point3
    #         else :
    #             return Point4

    def euclidean_distance(self, coord1, coord2):
        x1, y1, z1 = coord1
        x2, y2, z2 = coord2
        return ((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2) ** 0.5

    def generate_neighbors(self, current_coord, all_coords, distance_threshold):
        neighbors = []
        IsGateCenter = False
        IsCurrCoorOnCircle = current_coord in self.CircleCoord
        GateVar = -1
        for i in range(4):
            if current_coord[0] == self.GateList[i].x and current_coord[1] == self.GateList[i].y:
                IsGateCenter = True
                GateVar = self.GateList[i]
                break
        # print(current_coord)
        for coord in all_coords:
            CurrDist = self.euclidean_distance(current_coord, coord)
            if CurrDist <= distance_threshold:
                pts = self.generate_points(current_coord, coord, 30, self.ObstList, True)
                ClashCond = IsGateCenter and ((coord[0] == GateVar.entry2[0] and coord[1] == GateVar.entry2[1]) or (coord[0] == GateVar.exit2[0] and coord[1] == GateVar.exit2[1]))
                isCoordOnCircle = coord in self.CircleCoord
                IsDistanceOptimal = CurrDist < GlobalR
                if len(pts) < 30 or ClashCond or (IsCurrCoorOnCircle and isCoordOnCircle and IsDistanceOptimal):
                    continue
                neighbors.append(coord)
    
        return neighbors
    
    def reconstruct_path(self, came_from, current):
        total_path = [current]
        while current in came_from:
            current = came_from[current]
            total_path.append(current)
        return list(reversed(total_path))

    def astar(self, start, goal, all_coords):
        open_list = [(0, start)]  # Priority queue of (f-score, node)
        closed_set = set()
        came_from = {}
        g_score = {coord: float('inf') for coord in all_coords}
        g_score[start] = 0
        f_score = {coord: float('inf') for coord in all_coords}
        f_score[start] = self.euclidean_distance(start, goal)

        while open_list:
            _, current = heapq.heappop(open_list)

            if current == goal:
                return self.reconstruct_path(came_from, current)

            closed_set.add(current)

            for neighbor in self.generate_neighbors(current, all_coords, distance_threshold=2.5):
                if neighbor in closed_set:
                    continue

                tentative_g_score = g_score[current] + self.euclidean_distance(current, neighbor)

                if tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = g_score[neighbor] + self.euclidean_distance(neighbor, goal)
                    heapq.heappush(open_list, (f_score[neighbor], neighbor))

        return None  # No path found


    def generate_points(self, point1, point2, num_waypoints, ObstacleList, IsGateCheckEnabled):
        
        wp =[]
        for n in range(num_waypoints):
            ratio = n/(num_waypoints)
            x = point1[0] + ((point2[0]- point1[0])*ratio)
            y = point1[1] + ((point2[1]- point1[1])*ratio)
            IsObstacle = False
            for o in ObstacleList:
                if o[4] == 0:
                    if self.check_obstacle(x,y,o[0],o[1],GlobalR) :
                        IsObstacle = True
                        break
            IsCollidingWithGate = self.check_if_point_colliding_with_gate(x, y)
            if IsGateCheckEnabled :
                if (not IsObstacle) and (not IsCollidingWithGate):
                    wp.append((x,y,1))
            else :
                if (not IsObstacle) :
                    wp.append((x,y,1))
            
        return wp
    
    def check_if_point_colliding_with_gate(self, PointX, PointY) :
        GatesInfo = self.GateList
        for i in GatesInfo :
            Obs1 = i.obs_1
            Obs2 = i.obs_2
            Dist1 = math.sqrt((Obs1[0]-PointX)**2 + (Obs1[1]-PointY)**2)
            Dist2 = math.sqrt((Obs2[0]-PointX)**2 + (Obs2[1]-PointY)**2)
            if Dist1 <0.18 or Dist2 <0.18 :
                return True

        return False




    def planning(self, use_firmware, initial_info):
        """Trajectory planning algorithm"""
        #########################
        # REPLACE THIS (START) ##
        #########################
        ## generate waypoints for planning
        delta = 0.5
        num_waypoints = 50
        WP = []
        self.GateList = []
        path = []

        self.ObstList = self.obstacle_list(self.NOMINAL_OBSTACLES, self.NOMINAL_GATES)   
        self.CircleCoord = []
        for o in self.ObstList:
            if o[4] == 0:
                WP = WP + self.create_peripheral_coordinates_around_obstcl(o[0], o[1], GlobalR)
                self.CircleCoord = self.CircleCoord + self.create_peripheral_coordinates_around_obstcl(o[0], o[1], GlobalR)

        for i in self.NOMINAL_GATES :
            self.GateList.append(Gate(i[0],i[1], i[2], i[5], delta))

        
        for j in range(len(self.NOMINAL_GATES)):

            if j == 0:
                source_x = self.initial_obs[0]
                source_y = self.initial_obs[2]
                
            dest = self.GateList[j].calc_sequence(source_x , source_y)

            wp = self.generate_points([source_x, source_y], dest[0], num_waypoints, self.ObstList, True)

            WP = WP + wp

            nwp = self.generate_points(dest[0], dest[2], 10, self.ObstList, False)

            WP = WP + nwp
            for i in range(5):
                WP.append((dest[i][0],dest[i][1],1))

            
            path = path + self.astar((source_x,source_y, 1), (dest[0][0],dest[0][1],1), all_coords=WP)
            # path = path + nwp
            path = path + self.astar((dest[0][0],dest[0][1],1), (dest[1][0],dest[1][1],1), all_coords=WP)
            path = path + self.astar((dest[1][0],dest[1][1],1), (dest[2][0],dest[2][1],1), all_coords=WP)
            
            source_x = dest[2][0]
            source_y = dest[2][1]    

        # # Calculate the Euclidean distance between two points
        # def distance(x1, y1, x2, y2):
        #     return ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5

        # Initialize the closest points
        # closest_entry = None
        # closest_distance = float('inf')

        # Iterate over the entries
        # for i in (self.GateList):
        #     entry_x = i.entry[0]
        #     entry_y = i.entry[1]
        #     reference_x = initial_info["x_reference"][0]
        #     reference_y = initial_info["x_reference"][1]

        #     # Calculate the distance between the entry and the reference point
        #     dist = distance(entry_x, entry_y, reference_x, reference_y)

        #     # Update the closest points if a closer entry is found
        #     if dist < closest_distance:
        #         closest_gate = i
        #         closest_distance = dist

        # closest_entry is the entry closest to the reference point
        # print("Closest entry:", closest_entry)  
        # dest1 = closest_gate.calc_sequence(source_x , source_y)

        # path = path + self.astar((source_x,source_y, 1), (dest1[0][0],dest1[0][1],1), all_coords=WP)
        path = path + self.astar((source_x,source_y, 1),(initial_info["x_reference"][0], initial_info["x_reference"][2], 1), all_coords=WP)
        self.waypoints = np.array(path)
        # Call a function in module `example_custom_utils`.
        ecu.exampleFunction()

        # initial waypoint
        # if use_firmware:
        #     waypoints = [(self.initial_obs[0], self.initial_obs[2], initial_info["gate_dimensions"]["tall"]["height"])]  # Height is hardcoded scenario knowledge.
        # else:
        #     waypoints = [(self.initial_obs[0], self.initial_obs[2], self.initial_obs[4])]

        # # Example code: hardcode waypoints 
        # waypoints.append((-0.5, -3.0, 2.0))
        # waypoints.append((-0.5, -2.0, 2.0))
        # waypoints.append((-0.5, -1.0, 2.0))
        # waypoints.append((-0.5,  0.0, 2.0))
        # waypoints.append((-0.5,  1.0, 2.0))
        # waypoints.append((-0.5,  2.0, 2.0))
        # waypoints.append([initial_info["x_reference"][0], initial_info["x_reference"][2], initial_info["x_reference"][4]])

        # Polynomial fit.
        # self.waypoints = np.array(waypoints)
        deg = 12
        t = np.arange(self.waypoints.shape[0])
        # print("t",t)
        # fx = np.poly1d(np.polyfit(t, self.waypoints[:,0], deg))
        # fy = np.poly1d(np.polyfit(t, self.waypoints[:,1], deg))
        # fz = np.poly1d(np.polyfit(t, self.waypoints[:,2], deg))
        duration = 20
        # t_scaled = np.linspace(t[0], t[-1], int(duration*self.CTRL_FREQ))
        # self.ref_x = fx(t_scaled)
        # self.ref_y = fy(t_scaled)
        # self.ref_z = fz(t_scaled)

        spline_x = CubicSpline(t, self.waypoints[:, 0])
        spline_y = CubicSpline(t, self.waypoints[:, 1])
        spline_z = CubicSpline(t, self.waypoints[:, 2])

        # Interpolate along the splines to get the reference trajectory
        t_scaled = np.linspace(t[0], t[-1], int(duration * self.CTRL_FREQ))
        self.ref_x = spline_x(t_scaled)
        self.ref_y = spline_y(t_scaled)
        self.ref_z = spline_z(t_scaled)

        #########################
        # REPLACE THIS (END) ####
        #########################

        return t_scaled

    def cmdFirmware(self,
                    time,
                    obs,
                    reward=None,
                    done=None,
                    info=None
                    ):
        """Pick command sent to the quadrotor through a Crazyswarm/Crazyradio-like interface.

        INSTRUCTIONS:
            Re-implement this method to return the target position, velocity, acceleration, attitude, and attitude rates to be sent
            from Crazyswarm to the Crazyflie using, e.g., a `cmdFullState` call.

        Args:
            time (float): Episode's elapsed time, in seconds.
            obs (ndarray): The quadrotor's Vicon data [x, 0, y, 0, z, 0, phi, theta, psi, 0, 0, 0].
            reward (float, optional): The reward signal.
            done (bool, optional): Wether the episode has terminated.
            info (dict, optional): Current step information as a dictionary with keys
                'constraint_violation', 'current_target_gate_pos', etc.

        Returns:
            Command: selected type of command (takeOff, cmdFullState, etc., see Enum-like class `Command`).
            List: arguments for the type of command (see comments in class `Command`)

        """
        if self.ctrl is not None:
            raise RuntimeError("[ERROR] Using method 'cmdFirmware' but Controller was created with 'use_firmware' = False.")

        # [INSTRUCTIONS] 
        # self.CTRL_FREQ is 30 (set in the getting_started.yaml file) 
        # control input iteration indicates the number of control inputs sent to the quadrotor
        iteration = int(time*self.CTRL_FREQ)

        #########################
        # REPLACE THIS (START) ##
        #########################

        # print("The info. of the gates ")
        # print(self.NOMINAL_GATES)

        if iteration == 0:
            height = 1
            duration = 2

            command_type = Command(2)  # Take-off.
            args = [height, duration]

        # [INSTRUCTIONS] Example code for using cmdFullState interface   
        elif iteration >= 3*self.CTRL_FREQ and iteration < 30*self.CTRL_FREQ:
            step = min(iteration-3*self.CTRL_FREQ, len(self.ref_x) -1)
            target_pos = np.array([self.ref_x[step], self.ref_y[step], self.ref_z[step]])
            target_vel = np.zeros(3)
            target_acc = np.zeros(3)
            target_yaw = 0.
            target_rpy_rates = np.zeros(3)

            command_type = Command(1)  # cmdFullState.
            args = [target_pos, target_vel, target_acc, target_yaw, target_rpy_rates]

        elif iteration == 30*self.CTRL_FREQ:
            command_type = Command(6)  # Notify setpoint stop.
            args = []

       # [INSTRUCTIONS] Example code for using goTo interface 
        elif iteration == 30*self.CTRL_FREQ+1:
            x = self.ref_x[-1]
            y = self.ref_y[-1]
            z = 1.5 
            yaw = 0.
            duration = 2.5

            command_type = Command(5)  # goTo.
            args = [[x, y, z], yaw, duration, False]

        elif iteration == 33*self.CTRL_FREQ:
            x = self.initial_obs[0]
            y = self.initial_obs[2]
            z = 1.5
            yaw = 0.
            duration = 6

            command_type = Command(5)  # goTo.
            args = [[x, y, z], yaw, duration, False]

        elif iteration == 40*self.CTRL_FREQ:
            height = 0.
            duration = 3

            command_type = Command(3)  # Land.
            args = [height, duration]

        elif iteration == 33*self.CTRL_FREQ-1:
            command_type = Command(4)  # STOP command to be sent once the trajectory is completed.
            args = []

        else:
            command_type = Command(0)  # None.
            args = []

        #########################
        # REPLACE THIS (END) ####
        #########################

        return command_type, args

    def cmdSimOnly(self,
                   time,
                   obs,
                   reward=None,
                   done=None,
                   info=None
                   ):
        """PID per-propeller thrusts with a simplified, software-only PID quadrotor controller.

        INSTRUCTIONS:
            You do NOT need to re-implement this method for the project.
            Only re-implement this method when `use_firmware` == False to return the target position and velocity.

        Args:
            time (float): Episode's elapsed time, in seconds.
            obs (ndarray): The quadrotor's state [x, x_dot, y, y_dot, z, z_dot, phi, theta, psi, p, q, r].
            reward (float, optional): The reward signal.
            done (bool, optional): Wether the episode has terminated.
            info (dict, optional): Current step information as a dictionary with keys
                'constraint_violation', 'current_target_gate_pos', etc.

        Returns:
            List: target position (len == 3).
            List: target velocity (len == 3).

        """
        if self.ctrl is None:
            raise RuntimeError("[ERROR] Attempting to use method 'cmdSimOnly' but Controller was created with 'use_firmware' = True.")

        iteration = int(time*self.CTRL_FREQ)

        #########################
        if iteration < len(self.ref_x):
            target_p = np.array([self.ref_x[iteration], self.ref_y[iteration], self.ref_z[iteration]])
        else:
            target_p = np.array([self.ref_x[-1], self.ref_y[-1], self.ref_z[-1]])
        target_v = np.zeros(3)
        #########################

        return target_p, target_v

    def reset(self):
        """Initialize/reset data buffers and counters.

        Called once in __init__().

        """
        # Data buffers.
        self.action_buffer = deque([], maxlen=self.BUFFER_SIZE)
        self.obs_buffer = deque([], maxlen=self.BUFFER_SIZE)
        self.reward_buffer = deque([], maxlen=self.BUFFER_SIZE)
        self.done_buffer = deque([], maxlen=self.BUFFER_SIZE)
        self.info_buffer = deque([], maxlen=self.BUFFER_SIZE)

        # Counters.
        self.interstep_counter = 0
        self.interepisode_counter = 0

    # NOTE: this function is not used in the course project. 
    def interEpisodeReset(self):
        """Initialize/reset learning timing variables.

        Called between episodes in `getting_started.py`.

        """
        # Timing stats variables.
        self.interstep_learning_time = 0
        self.interstep_learning_occurrences = 0
        self.interepisode_learning_time = 0
