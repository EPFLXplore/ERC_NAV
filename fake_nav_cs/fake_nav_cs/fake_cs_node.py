import math
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
        super().__init__('Nav_fake_cs')
        self.position = [0,0,0]
        self.orientation = [0,0,0]
        self.linVel = [0,0,0]
        self.angVel = [0,0,0]

        self.steering_wheel_ang = [0,0,0,0]      # top left top right bottom left bottom right
        self.steering_wheel_state = [0,0,0,0]
        self.driving_wheel_ang = [0,0,0,0]
        self.driving_wheel_state = [0,0,0,0]
        self.text_entry_elements ={}

        self.create_subscription(Odometry,         '/lio_sam/odom',                self.nav_odometry  , 10)
        self.create_subscription(Wheelstatus,      '/NAV/absolute_encoders',       self.nav_wheel, 10)
        self.create_subscription(Motorcmds,        '/NAV/displacement',            self.nav_displacement, 10)
        self.display_ui()


    def nav_displacement(self, displacement):
        self.displacement_mode = displacement.modedeplacement
        self.info = displacement.info

        
    def nav_wheel(self, msg):
        """
        FRONT_LEFT_DRIVE = 0
        FRONT_RIGHT_DRIVE = 1
        BACK_RIGHT_DRIVE = 2
        BACK_LEFT_DRIVE = 3
        FRONT_LEFT_STEER = 4
        FRONT_RIGHT_STEER = 5
        BACK_RIGHT_STEER = 6
        BACK_LEFT_STEER = 7
        """
        print(msg.state)
        self.steering_wheel_ang = [float(i/65536 * 360) for i in msg.data[0:4]]
        # self.driving_wheel_ang = [float(i/65536 * 360) for i in msg.data[4:8]]
        self.steering_wheel_state = msg.state[0:4]
        self.driving_wheel_state = msg.state[4:8]
        self.text_elm_pos = []
        

    

    def nav_odometry(self, odometry):
        self.position = [odometry.pose.pose.position.x, odometry.pose.pose.position.y, odometry.pose.pose.position.z]

        orientation = quat2euler([odometry.pose.pose.orientation.w, 
                                    odometry.pose.pose.orientation.x, 
                                    odometry.pose.pose.orientation.y, 
                                    odometry.pose.pose.orientation.z])[2]
        # clamp the orientation from [-pi; pi] to between [0;2 pi]
        if (orientation < 0):
            orientation = orientation + 2* math.pi
        # convert the orientation from radians to degrees
        orientation = orientation * 180 / math.pi
        self.orientation = [0,0, orientation]

        self.linVel = [odometry.twist.twist.linear.x, odometry.twist.twist.linear.y, odometry.twist.twist.linear.z]
        self.angVel = [odometry.twist.twist.angular.x, odometry.twist.twist.angular.y, odometry.twist.twist.angular.z]

        self.update_ui()

    def display_ui(self):
        pygame.init()
        pygame.display.set_caption('Fake CS Rover Interface')
        self.window_surface = pygame.display.set_mode((1024, 768))
        self.background = pygame.Surface((1024, 768))
        self.background.fill(pygame.Color('#123456'))
        self.manager = pygame_gui.UIManager((1024, 768))
        label_layout_rect = pygame.Rect(30, 20, 100, 20)

        self.clock = pygame.time.Clock()
        is_running = True
      


        # Localization Panel Setup
        localization_panel = pygame_gui.elements.UIPanel(
            relative_rect=pygame.Rect(50, 50, 700, 200),
            manager=self.manager
        )
        loc_label = pygame_gui.elements.UILabel(relative_rect=label_layout_rect, text='Localization:',manager=self.manager, container = localization_panel)


        labels_loc = ['Position', 'Orientation', 'Rover linear speed', 'Rover angular speed']
        y_offset = 50
        label_height = 25
        text_entry_height = 25
        text_entry_width = 70
        spacing = 10

        # Generate Labels and Text Entry Fields for Localization
        for i, label in enumerate(labels_loc):
            # Calculate y position for label
            entries = []
            label_y = y_offset + (label_height + spacing) * i
            label_element = pygame_gui.elements.UILabel(
                relative_rect=pygame.Rect(10, label_y, 200, label_height),
                text=label,
                manager=self.manager,
                container=localization_panel
            )
            
            # Generate text entry fields for x, y, z components
            for j in range(3):
                text_entry_element = pygame_gui.elements.UITextEntryLine(
                    relative_rect=pygame.Rect(220 + j * (text_entry_width + spacing), label_y, text_entry_width, text_entry_height),
                    manager=self.manager,
                    container=localization_panel
                )
                text_entry_element.set_text('0')
                entries.append(text_entry_element)
            self.text_entry_elements[label] = entries


        # Wheels Panel Setup
        wheels_panel = pygame_gui.elements.UIPanel(
            relative_rect=pygame.Rect(50, 300, 700, 250),
            manager=self.manager
        )

        whls_label = pygame_gui.elements.UILabel(relative_rect=label_layout_rect, text='Wheels:',manager=self.manager, container = wheels_panel)


        wheels_labels = ['Wheels angle', 'Steering state', 'Driving state', ' Current']
        wheels_positions = ['Top-Left', 'Top-Right', 'Bottom-Left', 'Bottom-Right']

        # Generate Labels and Text Entry Fields for Wheels
        for i, pos in enumerate(wheels_positions):
            # Calculate y position for wheel label
            entries =[]
            wheel_label_y = y_offset + (label_height + spacing) * i
            wheel_label_element = pygame_gui.elements.UILabel(
                relative_rect=pygame.Rect(10, wheel_label_y, 200, label_height),
                text=pos,
                manager=self.manager,
                container=wheels_panel
            )
            
            # Generate text entry fields for each attribute of the wheel
            for j, attr in enumerate(wheels_labels):
                # Calculate x position for text entry
                text_entry_x = 220 + j * (text_entry_width + spacing)
                text_entry_element = pygame_gui.elements.UITextEntryLine(
                    relative_rect=pygame.Rect(text_entry_x, wheel_label_y, text_entry_width, text_entry_height),
                    manager=self.manager,
                    container=wheels_panel
                )
                text_entry_element.set_text('0')
                entries.append(text_entry_element)
            self.text_entry_elements[pos] = entries

        '''     # Main event loop
                while is_running:
                    for event in pygame.event.get():
                        if event.type == pygame.QUIT:
                            is_running = False
                        self.manager.process_events(event)
                    

        '''
        time_delta = self.clock.tick(60)/1000.0
        self.manager.update(time_delta)
        self.window_surface.blit(self.background, (0, 0))
        self.manager.draw_ui(self.window_surface)
        pygame.display.update()

        #pygame.quit()

    def update_ui(self):
        self.text_entry_elements['Position'][0].set_text(str(round(self.position[0],2)))  # Update X component
        self.text_entry_elements['Position'][1].set_text(str(round(self.position[1],2)))  # Update Y component
        self.text_entry_elements['Position'][2].set_text(str(round(self.position[2],2)))  # Update Z component
        
        self.text_entry_elements['Orientation'][0].set_text(str(round(self.orientation[0],2)))  
        self.text_entry_elements['Orientation'][1].set_text(str(round(self.orientation[1],2)))
        self.text_entry_elements['Orientation'][2].set_text(str(round(self.orientation[2],2)))

        self.text_entry_elements['Rover linear speed'][0].set_text(str(round(self.linVel[0],2)))  
        self.text_entry_elements['Rover linear speed'][1].set_text(str(round(self.linVel[1],2)))
        self.text_entry_elements['Rover linear speed'][2].set_text(str(round(self.linVel[2],2)))

        self.text_entry_elements['Rover angular speed'][0].set_text(str(round(self.angVel[0],2)))  
        self.text_entry_elements['Rover angular speed'][1].set_text(str(round(self.angVel[1],2)))
        self.text_entry_elements['Rover angular speed'][2].set_text(str(round(self.angVel[2],2)))

        self.text_entry_elements['Top-Left'][0].set_text(str(round(self.steering_wheel_ang[0],2)))
        self.text_entry_elements['Top-Left'][1].set_text(str(round(self.steering_wheel_state[0],2)))
        self.text_entry_elements['Top-Left'][2].set_text(str(round(self.driving_wheel_state[0],2)))
     
        self.text_entry_elements['Top-Right'][0].set_text(str(round(self.steering_wheel_ang[1],2)))
        self.text_entry_elements['Top-Right'][1].set_text(str(round(self.steering_wheel_state[1],2)))
        self.text_entry_elements['Top-Right'][2].set_text(str(round(self.driving_wheel_state[1],2)))

        self.text_entry_elements['Bottom-Left'][0].set_text(str(round(self.steering_wheel_ang[2],2)))
        self.text_entry_elements['Bottom-Left'][1].set_text(str(round(self.steering_wheel_state[2],2)))
        self.text_entry_elements['Bottom-Left'][2].set_text(str(round(self.driving_wheel_state[2],2)))

        self.text_entry_elements['Bottom-Right'][0].set_text(str(round(self.steering_wheel_ang[3],2)))
        self.text_entry_elements['Bottom-Right'][1].set_text(str(round(self.steering_wheel_state[3],2)))
        self.text_entry_elements['Bottom-Right'][2].set_text(str(round(self.driving_wheel_state[3],2)))                
        # top_left = [[self.steering_wheel_ang[0]] ,[self.steering_wheel_state[0]], [self.driving_wheel_ang[0]] ,[self.driving_wheel_state[0]]] 
        # top_right = [[self.steering_wheel_ang[1]] ,[self.steering_wheel_state[1]], [self.steering_wheel_state[1]], [self.driving_wheel_state[1]]] 
        # bottom_left = [[self.steering_wheel_ang[2]], [self.steering_wheel_state[2]], [self.steering_wheel_state[2]], [self.driving_wheel_state[2]]] 
        # bottom_right = [[self.steering_wheel_ang[3],] [self.steering_wheel_state[3]], [self.steering_wheel_state[3]], [self.driving_wheel_state[3]]]

        time_delta = self.clock.tick(60)/1000.0
        self.manager.update(time_delta)
        self.window_surface.blit(self.background, (0, 0))
        self.manager.draw_ui(self.window_surface)
        pygame.display.update()
        




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


'''
    text_entry_elements ={}
        position=[ self.position[0], self.position[1], self.position[2]  ]
        orientation=[self.orientation[0], self.orientation[1], self.orientation[2] ]
        rover_lin_sp =[self.linVel[0], self.linVel[1], self.linVel[2] ]
        rover_ang_sp =  [self.angVel[0], self.angVel[1], self.angVel[2]]
'''