#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import yaml

class PublisherInitialPose(Node):
    def __init__(self):
        super().__init__('pub_initial_pose_rover')
        self.publisher_ = self.create_publisher(PoseStamped, '/lio_sam/initial_pose', 10)

        self.take_zero_pose = True

        self.publish_initial_pose
        
        
        #self.timer = self.create_timer(1.0, self.publish_initial_pose)



    def get_initial_position_from_yaml(self,yaml_file_path):

        try:
            
            with open(yaml_file_path, 'r') as file:
                yaml_data = yaml.safe_load(file)
                
                initial_position = {
                    'px': yaml_data.get('px', 0.0),
                    'py': yaml_data.get('py', 0.0),
                    'pz': yaml_data.get('pz', 0.0),
                    'rx': yaml_data.get('rx', 0.0),
                    'ry': yaml_data.get('ry', 0.0),
                    'rz': yaml_data.get('rz', 0.0),
                }

                return initial_position

        except FileNotFoundError:
            self.get_logger().error('YAML file not found: %s', yaml_file_path)
            return None
        except Exception as e:
            self.get_logger().error('Error reading YAML file: %s', str(e))
            return None

    
    def publish_initial_pose(self):

        if self.take_zero_pose:
            initial_position = self.get_initial_position_from_yaml("initial_zero.yaml")
        else:
            initial_position = self.get_initial_position_from_yaml("initial_position.yaml")
        
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = 'map'
        pose_stamped.pose.position.x = initial_position['px']
        pose_stamped.pose.position.y = initial_position['py']
        pose_stamped.pose.position.z = initial_position['pz']
        pose_stamped.pose.orientation.x = initial_position['rx']
        pose_stamped.pose.orientation.y = initial_position['ry']
        pose_stamped.pose.orientation.z = initial_position['rz']
        pose_stamped.pose.orientation.w = 1.0  # Assuming unit quaternion for orientation
        
        self.publisher_.publish(pose_stamped)
        self.get_logger().info('Publishing initial pose rover')





def main(args=None):
    rclpy.init(args=args)
    initial_pose_publisher = PublisherInitialPose()
    rclpy.spin(initial_pose_publisher)
    initial_pose_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
