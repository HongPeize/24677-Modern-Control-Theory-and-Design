import numpy as np
import matplotlib.pyplot as plt
import math

from Astar import *
from util import *

class Planning(object):
    def __init__(self):
        self.track_file = 'buggyTrace.csv'
        self.costmap_file = 'buggy_costmap.npy'
        self.start_index = 0
        self.end_index = 2000
        self.resolution = 1
        self.tolerance = 5
        self.obstacle_start = 300
        self.obstacle_end = 900
        self.obstacle_length = 15
        self.obstacle_width = int(self.tolerance / 2)

    def convert_to_costmap(self, trajectory):
        '''
        @param trajectory: 2D numpy array of original trajectory
        return: 2D numpy array of 0 (freespace) and 1 (occupied space)
        '''
        coordinates = trajectory[self.start_index: self.end_index, :] - trajectory[self.start_index, :]
        #length = math.ceil(abs(coordinates[-1, 1]))
        #width = math.ceil(abs(coordinates[-1, 0]))
        length = math.ceil(max(abs(coordinates[:, 1])))
        width = math.ceil(max(abs(coordinates[:, 0])))

        costmap = np.ones((math.ceil(length / self.resolution), math.ceil(width / self.resolution)))

        for i in range(-self.tolerance, self.tolerance):
            x_coordinates = np.round(abs(coordinates[:, 1]) / self.resolution).astype('int')
            y_coordinates = np.round(abs(coordinates[:, 0]) / self.resolution).astype('int')
            self.clear_boundary(x_coordinates + i, y_coordinates, length, width, costmap)
            self.clear_boundary(x_coordinates, y_coordinates + i, length, width, costmap)

        return np.transpose(costmap)

    def clear_boundary(self, x_coordinates, y_coordinates, length, width, costmap):
        cord = np.hstack(
            (x_coordinates.reshape(len(x_coordinates), 1), y_coordinates.reshape(len(y_coordinates), 1)))
        cord = cord[0 <= cord[:, 0], :]
        cord = cord[cord[:, 0] < length, :]
        cord = cord[0 <= cord[:, 1], :]
        cord = cord[cord[:, 1] < width, :]
        costmap[cord[:, 0], cord[:, 1]] = 0

    def add_parked_vehicle(self, costmap, trajectory):
        '''
        @param costmap: original costmap
        @param trajectory: original trajectory
        return: costmap_vehicle: costmap with parked vehicle
        '''

        coordinates = trajectory[self.start_index: self.end_index, :] - trajectory[self.start_index, :]
        coordinates = coordinates[self.obstacle_start: self.obstacle_end, :]

        for i in range(-self.obstacle_width, self.obstacle_width):
            x_coordinates = np.round(abs(coordinates[:, 1]) / self.resolution).astype('int') + i
            y_coordinates = np.round(abs(coordinates[:, 0]) / self.resolution).astype('int')
            costmap[y_coordinates, x_coordinates] = 1

        for i in range(-self.obstacle_width, self.obstacle_width):
            x_coordinates = np.round(abs(coordinates[:, 1]) / self.resolution).astype('int')
            y_coordinates = np.round(abs(coordinates[:, 0]) / self.resolution).astype('int') + i
            costmap[y_coordinates, x_coordinates] = 1

        return costmap

    def convert_back(self, local_path, trajectory):
        '''
        @param local_path: local path (2D list) calculated by A*
        @param trajectory: original buggy track trajectory
        output: trajectory: global trajectory modified by local A* path
        '''
        local_path = np.array(local_path)
        astar_global_path = local_path * np.array([1, -1]) + trajectory[self.start_index, :]
        final_trajectory = np.vstack((trajectory[: self.start_index, :], astar_global_path, trajectory[self.end_index:, :]))

        return final_trajectory

    def run(self, file_name):
        # Load original track
        traj = getTrajectory(file_name)

        # Convert to costmap
        costmap = self.convert_to_costmap(traj)
        costmap = self.add_parked_vehicle(costmap, traj)
        np.save('buggy_costmap.npy', costmap)

        # A* planning
        Planner = AStar('buggy_costmap.npy')
        path = Planner.run(costmap, [0, 0], [costmap.shape[0] - 1, costmap.shape[1] - 1])
        visualizePath(costmap, path)

        # Update trajectory
        global_traj = self.convert_back(path, traj)

        x = [item[0] for item in global_traj]
        y = [item[1] for item in global_traj]
        plt.plot(x, y, "*", 'g')

        x = [item[0] for item in traj]
        y = [item[1] for item in traj]
        plt.plot(x, y, "*", 'r')
        plt.show()

        # Save global trajectory to csv
        np.savetxt("traj-astar.csv", global_traj, delimiter=",")

        return global_traj
