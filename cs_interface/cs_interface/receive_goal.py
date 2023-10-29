#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from cs_interface.cs_interface_node import BasicNavigator, NavigationResult
from nav2_msgs.action import NavigateToPose
from custom_msg.msg import NavFeedback
from builtin_interfaces.msg import Duration
class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('receive_goal')
        self.subscription = self.create_subscription(
            PoseStamped,
            '/ROVER/NAV_goal',
            self.goal_update,
            10)
        self.subscription2 = self.create_subscription(
             String,
            '/ROVER/NAV_status',
            self.status_update,
            10)
        # self.subscription  # prevent unused variable warning
        # self.subscription2
        self.goal=PoseStamped()
        self.navigator = BasicNavigator()
        print('ok')
        self.feedback = self.create_publisher(NavFeedback, '/NAV_feedback', 10)
        self.define_goals()
        self.path1= [
                [0.8, 1.82, 0.0, 0.0, 0.0, 0.707, 0.707],
                [1.13, 4.7, 0.0, 0.0, 0.0, 0.707, 0.707],               
                [1.13, 7.25, 0.0, 0.0, 0.0, 0.707, 0.707],                
                [0.66, 10.2, 0.0, 0.0, 0.0, 0.879, 0.477],                
                [-0.07, 12.61, 0.0, 0.0, 0.0, 0.766, 0.643],               
                [-0.41, 15.69, 0.0, 0.0, 0.0, 0.707, 0.707],               
                [0.19, 18.38, 0.0, 0.0, 0.0, 0.537, 0.843],               
                [2.34, 20.32, 0.0, 0.0, 0.0, 0.438, 0.899],              
                [4.55, 21.83, 0.0, 0.0, 0.0, 0.259, 0.966],               
                [7.9, 22.73, 0.0, 0.0, 0.0, 0.0, 0.0],               
                [11.12, 22.33, 0.0, 0.0, 0.0, -0.292, 0.956],               
                [12.45, 21.27, 0.0, 0.0, 0.0, 0.350, 0.937]]
        #timer_period = 10  # seconds
       # self.feedback.timer = self.create_timer(timer_period, self.timer_callback)
       # self.i = 0
    def define_goals(self):
         # waypoint 8
        self.target1 = PoseStamped()
        self.target1.header.frame_id = 'map'
        self.target1.pose.position.x = 12.45
        self.target1.pose.position.y = 21.27
        # waypoint 4
        self.target2 = PoseStamped()
        self.target2.header.frame_id = 'map'
        self.target2.pose.position.x = -6.99
        self.target2.pose.position.y = 25.22
        # waypoint 7
        self.target3 = PoseStamped()
        self.target3.header.frame_id = 'map'
        self.target3.pose.position.x = 13.85
        self.target3.pose.position.y = 10.86
        # waypoint 1
        self.target4 = PoseStamped()
        self.target4.header.frame_id = 'map'
        self.target4.pose.position.x = 3.83
        self.target4.pose.position.y = 9.10
        # Home
        self.target5 = PoseStamped()
        self.target5.header.frame_id = 'map'
        self.target5.pose.position.x = 0.0
        self.target5.pose.position.y = 0.0


    def status_update(self, msg):
        if (msg.data == "cancel"):
            print("cancel")
            self.navigator.cancelNav()

        if (msg.data=="abort"):
            print("abort")
            self.navigator.lifecycleShutdown()


    def goal_update(self, msg):
        # goal_poses = []
        # goal_pose = PoseStamped()
        # goal_pose.header.frame_id = 'map'
        # if(msg.pose.position.x==self.target1.pose.position.x and msg.pose.position.y==self.target1.pose.position.y ):
        #     for i in range(12):
        #         print("Goal 8 received")
        #         goal_pose.pose.position.x = self.path1[i][0]
        #         goal_pose.pose.position.y = self.path1[i][1]
        #         goal_pose.pose.position.z = self.path1[i][2]
        #         goal_pose.pose.orientation.x = self.path1[i][3]
        #         goal_pose.pose.orientation.y = self.path1[i][4]
        #         goal_pose.pose.orientation.z = self.path1[i][5]
        #         goal_pose.pose.orientation.w = self.path1[i][6]
        #         goal_poses.append(goal_pose)
        #     self.navigator.followWaypoints(goal_poses)
        #else:
        self.navigator.goToPose(msg)
def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()