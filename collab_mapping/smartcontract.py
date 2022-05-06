#! /usr/bin/env python
import os
import sys
import math
import random
import argparse
from dataclasses import dataclass

import numpy as np
from matplotlib import pyplot as plt

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid


UNKNOWN     = -1
SC_FREE     = -1
SC_UNKNOWN  = 0
SC_OCCUPIED = 1


def orientation_to_euler(self, orientation):
    x = orientation.x
    y = orientation.y
    z = orientation.z
    w = orientation.w
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return yaw_z # in radians


class ControllerAgent(Node) :

    def __init__(self, args) :

        # Init node
        super().__init__('controller_agent_{}'.format(random.randint(0,100000)))

        # Less than one third of them selected as Byzantine ones
        self.N = 8

        # Byzantine nodes and their corresponding anomilities that they wanna add
        self.byz_agents = {
            7: {
                "region": 3,
                "thickness": 20,
                "walls": [((18.5, 16.62), (20.7, 16.62)), ((18.5, 17.7), (20.7, 17.7))]
            },
            
            0: {
                "region": 2,
                "thickness": 180,
                "walls": [((-4.16, 12.9), (-3.16, 12.9))]
            }
        }

        # Subscribers
        self.map_subs = []
        for i in range(self.N):
            self.map_subs.append(self.create_subscription(OccupancyGrid, f'/robot{i}/map',
                self.make_cb(i), 10))
        self.mapping_done_sub = self.create_subscription(String, '/mapping_done', self.mapping_done_cb, 10)

        # Global Map Publisher
        self.glob_map_pub = self.create_publisher(OccupancyGrid, "/map", 10)
        self.timer = self.create_timer(20, self.glob_map_pub_cb)

        # State variable for storing each robot's map
        self.stored_maps = [None] * self.N

        # Add some tools and flags
        self.grid_tool = OccupancyGridTool(100, 100, 0.05, self.byz_agents, logger=self.get_logger())
        self.sc_agent = ByzTolerantMerge(2000, 2000, logger=self.get_logger())
        self.submitted_a_map = False

        # Initial poses of robots
        pkg_collab_mapping = get_package_share_directory('collab_mapping')
        self.init_poses = np.genfromtxt(os.path.join(pkg_collab_mapping, 'waypoints', 'init_poses.csv'), float, delimiter=',')

    def make_cb(self, index):
        robot_index = index
        def map_cb(msg):
            # Update the stored map for the current robot index
            self.stored_maps[robot_index] = msg
        return map_cb
    
    def glob_map_pub_cb(self):
        if not self.submitted_a_map:
            return
        
        global_map = self.sc_agent.getGlobalMap()
        msg = self.grid_tool.conv_sc_output_to_occupancygrid(global_map)
        self.get_logger().info("Publishing a global map ...")
        msg.header.stamp = self.get_clock().now().to_msg()
        self.glob_map_pub.publish(msg)
    
    def mapping_done_cb(self, msg):
        self.get_logger().info(f"Received a msg on mapping_done topic : {msg.data}")

        index = msg.data[5:]
        if not msg.data.startswith('robot') or not index.isdigit():
            self.get_logger().error("Mapping done messages should be like 'robot[i]' !!!")

        # Extract index of robot
        agent_index = int(index)

        # Check if robot index exists
        if agent_index >= self.N:
            self.get_logger().error(f"Index {agent_index} is out of range!")
            return

        map = self.stored_maps[agent_index]
        if map is None:
            self.get_logger().error(f"There is not submitted map from the robot {agent_index}!")
            return
        
        # Convert the latest stored map from robot to sc input
        conv_maps = self.grid_tool.conv_occupancygrid_to_sc_input(map, self.init_poses[agent_index,:])
        self.get_logger().info(f"There are {len(conv_maps)} submaps extraced from robot{agent_index} data.")
        
        # Apply byzantine behaviour
        final_maps = self.grid_tool.apply_byz_behaviour(conv_maps, agent_index)
        for rid, map in final_maps:
            self.submitted_a_map = True
            self.sc_agent.submitMap(map, rid, agent_index)
            self.get_logger().info(f"Robot{agent_index} submitted a map to SC!")


class OccupancyGridTool:
    def __init__(self, L, W, res, byz_agents, logger=None):
        '''
        OccupancyGridTool is responsible to read OccupancyGrid maps and
        convert them to the same coordinates.
        OccupancyGridTool will keep a 2-D matrix with size (NxM). The
        real-world pose of the cell (N/2, M/2) would be (0, 0).

        L: The global map length (m)
        W: The global map width (m)
        res: The map resolution (m/cell)
        '''

        self.L = L
        self.W = W
        self.res = res
        self.free_thresh = 50
        self.unknow_thresh = 0.7 * (L * W) / (4 * res * res)
        self.byz_agents = byz_agents
        self.logger = logger
    
    def apply_byz_behaviour(self, maps, agent_index):

        # No change for the ones that are not byzantine
        if agent_index not in self.byz_agents.keys():
            return maps
        
        final_maps = []
        anomalies = self.byz_agents[agent_index]
        for rid, map in maps:
            if anomalies["region"] == rid:
                thickness = anomalies["thickness"]
            
                # Insert walls
                for start, end in anomalies["walls"]:
                    w_start = int((start[0] % int(self.W / 2)) / self.res)
                    w_end = int((end[0] % int(self.W / 2)) / self.res)

                    if start[1] != end[1]:
                        raise Exception("Inserted anomaly walls should be horizontal!")

                    l = int((start[1] % int(self.L / 2)) / self.res)
                    for t in range(thickness):
                        map[(l + t) * int(self.W /self.res/ 2) + w_start: (l + t) * int(self.W /self.res/ 2) + w_end] = [SC_OCCUPIED] * (w_end - w_start)
                    self.logger.info(f"Inserted a wall into map submitted by robot{agent_index}!")
            
            final_maps.append((rid, map))

        return final_maps

    def conv_occupancygrid_to_sc_input(self, map, init_pose):
        '''
        This func converts an OccupancyGrid map to a SC input format.
        '''
        twod_arr = self._conv_occupancygrid_to_2d_array(map, init_pose)

        # Divide the 2d map into 4 regions
        regions = []
        regions.append(twod_arr[0:int(self.L/self.res/2), 0:int(self.W/self.res/2)])
        regions.append(twod_arr[0:int(self.L/self.res/2), int(self.W/self.res/2):int(self.W/self.res)])
        regions.append(twod_arr[int(self.L/self.res/2):int(self.L/self.res), 0:int(self.W/self.res/2)])
        regions.append(twod_arr[int(self.L/self.res/2):int(self.L/self.res), int(self.W/self.res/2):int(self.W/self.res)])

        # Check if the region has enough known values to worth submitting
        worthy_regions = []
        for i, region in enumerate(regions):
            self.logger.info(f"num unknow:{np.count_nonzero(region == UNKNOWN)}, threshold={self.unknow_thresh}")
            if np.count_nonzero(region == UNKNOWN) < self.unknow_thresh:
                worthy_regions.append((i, region))
        self.logger.info(f"Found {len(worthy_regions)} worthy regions in the map submitted!")

        # convert to 1d array and apply free threshold
        valid_regions = []
        for rid, region in worthy_regions:
            l, w = region.shape
            region = np.reshape(region, (l*w,))
            
            valid_region = (region == UNKNOWN) * SC_UNKNOWN + (region < self.free_thresh) * (region >= 0) * SC_FREE + (region >= self.free_thresh) * SC_OCCUPIED
            valid_regions.append((rid, valid_region))
        
        return valid_regions


    def _conv_occupancygrid_to_2d_array(self, map, init_pose):
        '''
        This func converts an OccupancyGrid map to a 2-D matrix.
        '''

        # Check if the resolution matches
        if self.res != np.round(map.info.resolution, 2):
            raise Exception(f"The resolution of the map is not matching! (map_res={map.info.resolution},base_res={self.res})")

        # Extract map info
        w = map.info.width
        l = map.info.height
        origin_position = map.info.origin.position
        map_ = np.reshape(map.data, (l, w))
        origin_w = int((origin_position.x + init_pose[0] + self.W // 2) // self.res)
        origin_l = int((origin_position.y + init_pose[1] + self.L // 2) // self.res)
        global_map = np.ones((int(self.L / self.res), int(self.W / self.res))) * -1

        # Put the map in global map
        for l_ in range(l):
            for w_ in range(w):
                if origin_l + l_ >= self.L // self.res or origin_w + w_ >= self.W // self.res:
                    continue
                if origin_l + l_ < 0 or origin_w + w_ < 0:
                    continue
                global_map[origin_l + l_, origin_w + w_] = map_[l_, w_]
        
        return global_map
    
    def conv_sc_output_to_occupancygrid(self, map_cells):
        map = OccupancyGrid()
        map.header.frame_id = "map"
        map.info.resolution = 0.05
        map.info.width = int(self.W / self.res)
        map.info.height = int(self.L / self.res)
        map.data = [int(val) for val in (map_cells == SC_UNKNOWN) * UNKNOWN + (map_cells == SC_OCCUPIED) * 100 + (map_cells == SC_FREE) * 0]

        return map


class ByzTolerantMerge:
    @dataclass
    class Map:
        cells:      list
        conflict:   int
        comply:     int
        rid:        int # Region ID
        uid:        int # Agent ID

    def __init__(self, length, width, logger=None):
        # The whole area size in pixels
        self.length = length
        self.width = width

        # We divide the whole area into 4 regions
        self.N = 4
        self.RL = int(self.length / 2)
        self.RW = int(self.width / 2)

        # In each area we will iterate over windows of size:
        self.WW = 50
        self.WL = 50

        self.agents = [True] * 8
        self.submittedMaps = list()
        self.logger = logger
        self.byz_threshold = 0.2

    def submitMap(self, newMapCells, rid, uid):
        newMap = ByzTolerantMerge.Map(newMapCells, 0, 0, rid, uid)

        # Iterate over all previously submitted maps
        for map in self.submittedMaps:
            # Skip if it's not the same region
            if map.rid != rid:
                continue

            # Compare the region map and increament if it complies or conflicts
            if self.compareMap(map.cells, newMap.cells):
                map.comply += 1
                newMap.comply += 1
            else:
                map.conflict += 1
                newMap.conflict += 1
            
            # See if the coresponding agents are byzantine or not
            for map_ in [map, newMap]:
                if map_.conflict + map_.comply >= 3 and map_.conflict > map_.comply:
                    self.logger.info(f"Robot{map_.uid} is detected as byzantine and removing all maps from them!")
                    self.agents[map_.uid] = False
        
        # Append to the list of submitted maps
        self.submittedMaps.append(newMap)
            
    def compareMap(self, map1, map2):
        byz = True
        for L in range(self.RL // self.WL):
            if not byz:
                break

            for W in range(self.RW // self.WW):
                similarity = 0
                diff = 0
                z1 = 0
                z2 = 0
                
                for l in range(self.WL):
                    start = (L * self.WL + l) * self.RW + W * self.WW
                    end = (L * self.WL + l) * self.RW + (W + 1) * self.WW
                    similarity += sum([1 for i in range(self.WW) if (map1[start + i] == map2[start + i] and map1[start + i] != SC_UNKNOWN)])
                    diff += sum([1 for i in range(self.WW) if (map1[start + i] != map2[start + i] and map1[start + i] != SC_UNKNOWN and map2[start + i] != SC_UNKNOWN)])
                    z1 += np.count_nonzero(map1[start:end] == SC_UNKNOWN)
                    z2 += np.count_nonzero(map2[start:end] == SC_UNKNOWN)

                if diff + similarity == 0 or z1/self.WL/self.WW > 0.2 or z2/self.WL/self.WW > 0.2:
                    # self.logger.info(f'L = {L}, W = {W} Z1 = {z1}, z2 = {z2}: No match!')
                    pass
                else:
                    if diff/(diff+similarity) > self.byz_threshold:
                        byz = False
                        self.logger.info(f'L = {L}, W = {W} : {diff/(diff+similarity)} => detected as byz!')
                        break
                    # self.logger.info(f'L = {L}, W = {W} : {diff/(diff+similarity)}')
        
        return byz

    def getGlobalMap(self):
        globMap = np.zeros(self.length * self.width)
        self.logger.info("Started to create a global map ...")
        for map in self.submittedMaps:
            # Skip if the agent is byzantine
            if not self.agents[map.uid]:
                continue

            if map.comply >= map.conflict:
                self.logger.info("Including a sub-map in global one ...")
                
                for l in range(self.RL):
                    start = l * self.width + (map.rid & 1) * self.RW + ((map.rid & 2) >> 1) * self.width * self.RL
                    end = start + self.RW
                    globMap[start:end] += map.cells[l * self.RW: (l + 1) * self.RW]

        return np.sign(globMap)
        
        
def main(args=None):
    rclpy.init(args=args)
    args_without_ros = rclpy.utilities.remove_ros_args(args)

    controller = ControllerAgent(args_without_ros)
    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
