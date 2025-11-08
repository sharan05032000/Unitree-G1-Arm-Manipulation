import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive

class MoveGroupClient(Node):

    def __init__(self):
        super().__init__('move_group_client')
        self._action_client = ActionClient(self, MoveGroup, '/move_action')
        self.planning = None
       
        
    def send_pose_goal(self, group_name, target_pose_stamped):
        if not self.planning:   
            self.planning = True

            self.get_logger().info('Waiting for action server...')
            self._action_client.wait_for_server()

            goal_msg = MoveGroup.Goal()
            goal_msg.request.group_name = group_name

            # Create position constraint with bounding sphere of radius 1cm
            position_constraint = PositionConstraint()
            position_constraint.header = target_pose_stamped.header
            position_constraint.link_name = 'right_wrist_yaw_link'  # from your SRDF end effector
            position_constraint.weight = 1.0
            position_constraint.target_point_offset.x = 0.0
            position_constraint.target_point_offset.y = 0.0
            position_constraint.target_point_offset.z = 0.0

            sphere = SolidPrimitive()
            sphere.type = Sphere = SolidPrimitive.SPHERE
            sphere.dimensions = [0.001]  # 1 cm radius sphere

            position_constraint.constraint_region.primitives = [sphere]
            position_constraint.constraint_region.primitive_poses = [target_pose_stamped.pose]

            # Create orientation constraint with some tolerance
            orientation_constraint = OrientationConstraint()
            orientation_constraint.header = target_pose_stamped.header
            orientation_constraint.orientation = target_pose_stamped.pose.orientation
            orientation_constraint.link_name = 'right_wrist_yaw_link'
            orientation_constraint.absolute_x_axis_tolerance = 0.001
            orientation_constraint.absolute_y_axis_tolerance = 0.001
            orientation_constraint.absolute_z_axis_tolerance = 0.001
            orientation_constraint.weight = 1.0

            # Bundle constraints
            goal_constraint = Constraints()
            goal_constraint.position_constraints = [position_constraint]
            goal_constraint.orientation_constraints = [orientation_constraint]

            goal_msg.request.goal_constraints = [goal_constraint]

            # Optional: Add some planning parameters to improve success
            goal_msg.request.allowed_planning_time = 5.0
            goal_msg.request.num_planning_attempts = 5

            self.get_logger().info('Sending goal to MoveGroup action server...')
            future = self._action_client.send_goal_async(goal_msg)
            future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected by action server.')
            self.planning = False
            return

        self.get_logger().info('Goal accepted by action server.')
        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.error_code.val == result.error_code.SUCCESS:
            self.get_logger().info('Goal succeeded!')
        else:
            self.get_logger().error(f'Goal failed with error code: {result.error_code.val}')
        self.planning = False

       

def main(args=None):
    rclpy.init(args=args)
    node = MoveGroupClient()
    try:
        while True:
            if not node.planning or node.planning is None:
                target_pose = PoseStamped()
                target_pose.header.frame_id = 'pelvis'  # from your SRDF virtual joint
                target_pose.header.stamp = node.get_clock().now().to_msg()
                x=float(input("Enter x :"))
                y=float(input("Enter y :"))
                z=float(input("Enter z :"))
                target_pose.pose.position.x = x
                target_pose.pose.position.y = y
                target_pose.pose.position.z = z
                target_pose.pose.orientation.w = 1.0  # identity orientation
                node.send_pose_goal('right', target_pose)
            rclpy.spin_once(node)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


