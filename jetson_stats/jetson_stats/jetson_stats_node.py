import rclpy
from rclpy.node import Node
import time, json
from jtop import jtop

from std_msgs.msg import String, Float32MultiArray, Float32, Bool
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup

class JetsonStatsTracker(Node):

    def __init__(self):
        super().__init__('JetsonStatsTrackerNAV')
        
        self.callback_group = ReentrantCallbackGroup()
        
        self.jtop = jtop()
        self.jtop.start()

        # Because the stats don't need to be accurate, we provide the minimal setup for the publisher
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT, # BEST_EFFORT: message will attempt to send message but if it fails it will not try again
            durability=QoSDurabilityPolicy.VOLATILE, # VOLATILE: if no subscribers are listening, the message sent is not saved
            history=QoSHistoryPolicy.KEEP_LAST, # KEEP_LAST: only the last n = depth messages are stored in the queue
            depth=1,
        )
        
        self.json_stats_obj = {}
        
        self.publisher_stats = self.create_publisher(String, '/NAV/jetson_stats', self.qos_profile, callback_group=self.callback_group)
        
        self.timer_stats = self.create_timer(3.0, self.stats)
        
    def stats(self):                  
        try:          
            # jetson.ok() will provide the proper update frequency
            if self.jtop is None or not self.jtop.ok(spin=True):
                return

            jetson = self.jtop
            msg = String()
            
            ## CPU: we have 8 of them
            ## -> user (percentage of utilization)
            utilization_cpus = [round(obj.get('user', 0.0), 1) for obj in jetson.cpu['cpu']]
            self.json_stats_obj['utilization_cpus'] = utilization_cpus

            ## MEMORY
            ## -> ram.used in KB => convert in GB
            ram_val = round((jetson.memory['RAM']['used'] / 1000000), 1)
            self.json_stats_obj['ram'] = ram_val

            ## GPU
            ## -> status.load
            load_gpu = jetson.gpu['gpu']['status']['load']
            self.json_stats_obj['load_gpu'] = load_gpu

            ## FAN: we have 1 of them
            ## -> rpm
            fan_rpm = jetson.fan['pwmfan']['rpm'][0]
            self.json_stats_obj['fan_rpm'] = fan_rpm

            ## POWER
            ## -> tot.power in milliwatt
            power_tot = round((jetson.power['tot']['curr'] / 1000), 1)
            self.json_stats_obj['power_tot'] = power_tot

            ## temperature: multiple parts of the board, but take only cpu and gpu
            ## -> cpu.temp in celcius
            ## -> gpu.temp in celcius
            temp_cpu = round(jetson.temperature['cpu']['temp'], 1)
            temp_gpu = round(jetson.temperature['gpu']['temp'], 1)
            self.json_stats_obj['temp_cpu'] = temp_cpu
            self.json_stats_obj['temp_gpu'] = temp_gpu
            #self.get_logger().info(f"{self.json_stats_obj}")
            msg.data = json.dumps(self.json_stats_obj)
            self.publisher_stats.publish(msg)

        except Exception as e:
            self.get_logger().warning(f"Error in stats: {e}")
            return

    def close(self):
        if self.jtop is not None:
            self.jtop.close()
            self.jtop.join(timeout=1.0)
            self.jtop = None


def main(args=None):
    rclpy.init(args=args)

    executor = rclpy.executors.SingleThreadedExecutor()
    stats_tracker = JetsonStatsTracker()
    executor.add_node(stats_tracker)    
    
    try:
        executor.spin()
    finally:
        executor.shutdown()
        try:
            stats_tracker.close()
        finally:
            stats_tracker.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()
    
    
if __name__ == '__main__':
    main()
