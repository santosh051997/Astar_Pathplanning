#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
import numpy as np
from .occupancygridparam import OccupaancyGridParam
from .astar import Astar, AstarConfig


class GlobalPlanner(Node):

	def __init__(self):
		super().__init__("global_planner")

		# Parameters Declaration
		self.declare_parameter("rate", 10)
		self.declare_parameter("euclidean", True)
		self.declare_parameter("occupyThresh", -1)
		self.declare_parameter("inflateRadius", 1.0)

		self.rate = self.get_parameter("rate").get_parameter_value().integer_value
		self.euclidean = self.get_parameter("euclidean").get_parameter_value().bool_value
		self.occupyThresh = self.get_parameter("occupyThresh").get_parameter_value().integer_value
		self.inflateRadius = self.get_parameter("inflateRadius").get_parameter_value().double_value

		# Instance variables
		self.occGridParam = OccupaancyGridParam()
		self.astar = Astar()
		self.inflateMap = OccupancyGrid()

		# Flag
		self.startFlag = False
		self.startPointFlag = False
		self.targetPointFlag = False
      
		# Publishers
		self.map_pub = self.create_publisher(OccupancyGrid, 'inflate_map', 10)
		self.path_pub = self.create_publisher(Path, 'nav_path', 10)

		# Subscribers
		self.initialpose_subscriber = self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.startPointCallback, 10)
		self.goalpose_subscriber = self.create_subscription(PoseStamped, '/goal_pose', self.targetPointtCallback, 10)
		
		# Map service
		self.occGrid = None
		self.map_client = self.create_client(GetMap, '/map_server/map')
        
		while not self.map_client.wait_for_service(timeout_sec=10.0):
			self.get_logger().info('Waiting for service')
		
		self.setup()
					
	def setup(self):

		self.map_request = GetMap.Request()
		#self.map_request.map_url = "/home/santosh/ros2mapping_ws/src/astar_pathplanning/maps/local_roadmap.yaml"
		wait = self.map_client.call_async(self.map_request)
		rclpy.spin_until_future_complete(self, wait)
		if wait.result() is not None:
			self.get_logger().info('Request was responded')
			response = wait.result()
			self.occGrid = response.map
		else:
			self.get_logger().info('request Failed')
			return False

		inflateRadius = int(round(self.inflateRadius / self.occGrid.info.resolution))
		self.occGridParam.setOccupancyGridParam(self.occGrid)
		occMap = self.occGridParam.getMap()
		occMap_copy = np.array(occMap)
		occMap_copy[occMap_copy < 0] = 100
		occMap_copy = 100 - np.uint8(occMap_copy)
		config = AstarConfig(self.euclidean, self.occupyThresh, inflateRadius)
		infMap = self.astar.initAstar(occMap_copy, config)
		infMap = np.int8(np.clip(infMap, 0, 100))
		self.inflateMap = self.occGrid
		
		self.inflateMap.data = infMap.flatten().tolist()

		return True
	
	def spin(self):

		# rate = self.create_rate(self.rate)
		while rclpy.ok():
            # Publish inflate map
			self.publishInflateMap()
			
			if self.startFlag:
                # Start planning path
				pathList = self.astar.pathPlanning(self.startPoint, self.targetPoint)
				if len(pathList):
					path = Path()
					path.header.stamp = self.get_clock().now().to_msg()
					path.header.frame_id = 'map'
					pathList = self.occGridParam.image2MapTransform(pathList)
					for point in pathList:
						poseStamped = PoseStamped()
						poseStamped.header.stamp = self.get_clock().now().to_msg()
						poseStamped.header.frame_id = 'map'
						poseStamped.pose.position.x = float(point[0])
						poseStamped.pose.position.y = float(point[1])
						poseStamped.pose.position.z = 0.0
						path.poses.append(poseStamped)
					self.path_pub.publish(path)
					self.get_logger().info('Found a valid path successfully')
				else:
					self.get_logger().error('Cannot find a valid path')
					self.startFlag = False

			self.spin_once()
					
			# rate.sleep()

	def spin_once(self):
		rclpy.spin_once(self)

	def startPointCallback(self, msg):
		startPoint = [msg.pose.pose.position.x, msg.pose.pose.position.y]
		startPoint = self.occGridParam.map2ImageTransform([startPoint])[0]
		startPoint = np.int32(np.round(startPoint))
		self.startPoint = [startPoint[0], startPoint[1]]
		self.startPointFlag = True
		self.startFlag = self.startPointFlag and self.targetPointFlag

		self.get_logger().info("startPoint: %f, %f, %d, %d" % (msg.pose.pose.position.x, msg.pose.pose.position.y, startPoint[0], startPoint[1]))

	def targetPointtCallback(self, msg):
		targetPoint = [msg.pose.position.x, msg.pose.position.y]
		targetPoint = self.occGridParam.map2ImageTransform([targetPoint])[0]
		targetPoint = np.int32(np.round(targetPoint))
		self.targetPoint = [targetPoint[0], targetPoint[1]]
		self.targetPointFlag = True
		self.startFlag = self.startPointFlag and self.targetPointFlag
		
		self.get_logger().info("targetPoint: %f, %f, %d, %d" % (msg.pose.position.x, msg.pose.position.y, targetPoint[0], targetPoint[1]))
    
	def publishInflateMap(self):
		self.inflateMap.header.stamp = self.get_clock().now().to_msg()
		self.map_pub.publish(self.inflateMap)

def main(args=None):
	rclpy.init(args=args)
	global_planner = GlobalPlanner()
	global_planner.spin()
	rclpy.shutdown()
	
if __name__ == '__main__':
    main()
  
