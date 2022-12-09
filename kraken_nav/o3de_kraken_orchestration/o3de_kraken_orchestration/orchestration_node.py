from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from gazebo_msgs.srv import GetModelState
from nav_msgs.srv import GetPlan
from std_srvs.srv import Trigger, Empty
from rcl_interfaces.msg import Log
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from std_msgs.msg import String
from std_msgs.msg import Float32


class KrakenOrchestrationNode(Node):
    def init_spawn_entity_client(self):
        self.spawn_entity_client = self.create_client(SpawnEntity, 'spawn_entity')
        while not self.spawn_entity_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Spawn entity service not available waiting again...')

    def init_get_spawn_point_info_client(self):
        self.get_spawn_point_info_client = self.create_client(GetModelState, 'get_spawn_point_info')
        while not self.get_spawn_point_info_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Get spawn point info service not available waiting again...')

    def init_get_plan_client(self):
        self.get_plan_client = self.create_client(GetPlan, 'get_gathering_plan')
        while not self.get_plan_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Get plan service not available waiting again...')

    def apple_gathering_done_callback(self, request, response):
        self.get_logger().info("Received done client msg.")
        self.gathering_done = True
        return Empty.Response()

    def apple_progress_callback(self, msg):
        self.apple_progress = 100.0 * float(msg.data)

    def init_apple_gathering_srvs(self):
        self.done_apple_gathering_service = self.create_service(Empty, f'{self.kraken_name}/done_apple_gathering',
                                                                self.apple_gathering_done_callback)

        self.trigger_apple_gathering_client = self.create_client(Trigger, f'{self.kraken_name}/trigger_apple_gathering')
        self.cancel_apple_gathering_client = self.create_client(Trigger, f'{self.kraken_name}/cancel_apple_gathering')

        while (not self.trigger_apple_gathering_client.wait_for_service(timeout_sec=1.0)) \
                and (not self.cancel_apple_gathering_client.wait_for_service(timeout_sec=1.0)):
            self.get_logger().info('Apple gathering services not available waiting again...')

    def init_apple_subscriptions(self):
        qos = QoSProfile(
            depth=5,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        self.progress_apple_gathering_sub = self.create_subscription(Float32,
                                                                     f'{self.kraken_name}/progress_apple_gathering',
                                                                     self.apple_progress_callback, qos)

    def init_goal_pose_publisher(self):
        qos = QoSProfile(
            depth=5,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        self.goal_pose_publisher = self.create_publisher(PoseStamped, f'{self.kraken_name}/goal_pose', qos)

    def init_status_publisher(self):
        qos = QoSProfile(
            depth=5,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        self.status_publisher = self.create_publisher(String, f'{self.kraken_name}/orchestration_status', qos)

    def send_state(self, status_str):
        msg = String()
        msg.data = status_str
        self.status_publisher.publish(msg)

    def __init__(self):
        super().__init__('kraken_orchestration_node')

        self.declare_parameter('robot_name', 'apple_kraken_rusty_1')
        self.kraken_name = self.get_parameter('robot_name').get_parameter_value().string_value

        self.declare_parameter('spawn_line', 'line1')
        self.spawn_point = self.get_parameter('spawn_line').get_parameter_value().string_value

        self.log_sub = self.create_subscription(Log, "/rosout", self.log_cb, 10)
        self.goal_pose_reached = False
        self.gathering_done = False
        self.apple_progress = 0.0

        self.init_spawn_entity_client()
        self.init_get_spawn_point_info_client()
        self.init_get_plan_client()
        self.init_goal_pose_publisher()
        self.init_status_publisher()
        self.init_apple_subscriptions()

    def log_cb(self, msg):
        if msg.msg == "Goal succeeded" and self.kraken_name in str(msg.name):
            self.get_logger().info("Navigation point reached.")
            self.goal_pose_reached = True

    def spawn_kraken(self):
        req = SpawnEntity.Request()
        req.name = self.kraken_name[:len(self.kraken_name) - 2]
        req.xml = self.spawn_point

        res = self.spawn_entity_client.call(req)  # TODO - Add exception handling
        if res.success:
            self.init_apple_gathering_srvs()

        return res.success

    def get_spawn_point_pose(self):
        req = GetModelState.Request()
        req.model_name = self.spawn_point

        res = self.get_spawn_point_info_client.call(req)

        return res.pose  # TODO - Add exception handling

    def get_plan_poses(self, pose):
        req = GetPlan.Request()
        req.start.pose = pose

        res = self.get_plan_client.call(req)
        return res.plan.poses

    def trigger_apple_gathering(self):
        self.gathering_done = False
        req = Trigger.Request()
        return self.trigger_apple_gathering_client.call(req)

    def cancel_apple_gathering(self):
        req = Trigger.Request()
        return self.cancel_apple_gathering_client.call(req)

    def navigate_to_pose(self, pose):
        pose.header = Header(stamp=self.get_clock().now().to_msg(), frame_id='map')
        self.goal_pose_publisher.publish(pose)
