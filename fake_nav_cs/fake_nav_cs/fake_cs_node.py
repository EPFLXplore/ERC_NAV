import rclpy
from rclpy.node import Node
from transforms3d.euler import quat2euler
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from custom_msg.msg import Wheelstatus, Motorcmds
import pygame
import pygame_gui



# Node for the Nav fake CS
class Fake_cs_nav(Node):

    def __init__(self):
        super().__init__('/Nav/fake_cs')
        self.position = [0,0,0]
        self.orientation = [0,0,0]
        self.linVel = [0,0,0]
        self.angVel = [0,0,0]

        self.steering_wheel_ang = [0,0,0,0]
        self.steering_wheel_state = [0,0,0,0]
        self.driving_wheel_ang = [0,0,0,0]
        self.driving_wheel_state = [0,0,0,0]

        self.create_subscription(Odometry,         '/lio_sam/odom',                self.nav_odometry  , 10)
        self.create_subscription(Motorcmds,        '/NAV/displacement',            self.nav_displacement, 10)
        self.display_ui()


    def nav_displacement(self, displacement):
        self.navigation.displacement_mode = displacement.modedeplacement
        self.navigation.info = displacement.info
    

    def nav_odometry(self, odometry):
        self.navigation.position = [odometry.pose.pose.position.x, odometry.pose.pose.position.y, odometry.pose.pose.position.z]

        orientation = quat2euler([odometry.pose.pose.orientation.w, 
                                    odometry.pose.pose.orientation.x, 
                                    odometry.pose.pose.orientation.y, 
                                    odometry.pose.pose.orientation.z])[2]
        # clamp the orientation from [-pi; pi] to between [0;2 pi]
        if (orientation < 0):
            orientation = orientation + 2* np.pi
        # convert the orientation from radians to degrees
        orientation = orientation * 180 / np.pi
        self.navigation.orientation = [0,0, orientation]

        self.navigation.linVel = [odometry.twist.twist.linear.x, odometry.twist.twist.linear.y, odometry.twist.twist.linear.z]
        self.navigation.angVel = [odometry.twist.twist.angular.x, odometry.twist.twist.angular.y, odometry.twist.twist.angular.z]

    def display_ui():
        pygame.init()
        pygame.display.set_caption('Fake CS Rover Interface')
        window_surface = pygame.display.set_mode((1024, 768))
        background = pygame.Surface((1024, 768))
        background.fill(pygame.Color('#123456'))
        manager = pygame_gui.UIManager((1024, 768), 'theme.json')
        label_layout_rect = pygame.Rect(30, 20, 100, 20)

        clock = pygame.time.Clock()
        is_running = True

        # Localization Panel Setup
        localization_panel = pygame_gui.elements.UIPanel(
            relative_rect=pygame.Rect(50, 50, 700, 200),
            manager=manager
        )
        loc_label = pygame_gui.elements.UILabel(relative_rect=label_layout_rect, text='Localization:',manager=manager, container = localization_panel)


        labels_loc = ['Position', 'Orientation', 'Rover linear speed', 'Rover angular speed']
        y_offset = 50
        label_height = 25
        text_entry_height = 25
        text_entry_width = 70
        spacing = 10

        # Generate Labels and Text Entry Fields for Localization
        for i, label in enumerate(labels_loc):
            # Calculate y position for label
            label_y = y_offset + (label_height + spacing) * i
            label_element = pygame_gui.elements.UILabel(
                relative_rect=pygame.Rect(10, label_y, 200, label_height),
                text=label,
                manager=manager,
                container=localization_panel
            )
            
            # Generate text entry fields for x, y, z components
            for j in range(3):
                text_entry_element = pygame_gui.elements.UITextEntryLine(
                    relative_rect=pygame.Rect(220 + j * (text_entry_width + spacing), label_y, text_entry_width, text_entry_height),
                    manager=manager,
                    container=localization_panel
                )
                text_entry_element.set_text('0')

        # Wheels Panel Setup
        wheels_panel = pygame_gui.elements.UIPanel(
            relative_rect=pygame.Rect(50, 300, 700, 250),
            manager=manager
        )

        whls_label = pygame_gui.elements.UILabel(relative_rect=label_layout_rect, text='Wheels:',manager=manager, container = wheels_panel)


        wheels_labels = ['Speed', 'Steering', 'Current', 'Motor state']
        wheels_positions = ['Top-Left', 'Top-Right', 'Bottom-Left', 'Bottom-Right']

        # Generate Labels and Text Entry Fields for Wheels
        for i, pos in enumerate(wheels_positions):
            # Calculate y position for wheel label
            wheel_label_y = y_offset + (label_height + spacing) * i
            wheel_label_element = pygame_gui.elements.UILabel(
                relative_rect=pygame.Rect(10, wheel_label_y, 200, label_height),
                text=pos,
                manager=manager,
                container=wheels_panel
            )
            
            # Generate text entry fields for each attribute of the wheel
            for j, attr in enumerate(wheels_labels):
                # Calculate x position for text entry
                text_entry_x = 220 + j * (text_entry_width + spacing)
                text_entry_element = pygame_gui.elements.UITextEntryLine(
                    relative_rect=pygame.Rect(text_entry_x, wheel_label_y, text_entry_width, text_entry_height),
                    manager=manager,
                    container=wheels_panel
                )
                text_entry_element.set_text('0')

        # Main event loop
        while is_running:
            time_delta = clock.tick(60)/1000.0
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    is_running = False
                manager.process_events(event)
            
            manager.update(time_delta)
            window_surface.blit(background, (0, 0))
            manager.draw_ui(window_surface)
            pygame.display.update()

        pygame.quit()




def main(args=None):
    rclpy.init(args=args)

    fake_cs_nav = Fake_cs_nav()

    rclpy.spin(fake_cs_nav)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    fake_cs_nav.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


