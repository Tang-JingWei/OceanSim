import time
import os
import numpy as np
from enum import Enum

from isaacsim.core.prims import SingleRigidPrim
from isaacsim.core.utils.prims import get_prim_path

'''
Attention:

Before OceanSim extension being activated, the extension isaacsim.ros2.bridge should be activated, otherwise rclpy will
fail to be loaded.

so, we suggest that make sure the extension isaacsim.ros2.bridge is being setup to "AUTOLOADED" in Window->Extension.
'''
import rclpy
from geometry_msgs.msg import Twist, Wrench, PoseStamped
            
try:
    from pxr import Gf, PhysxSchema
    PXR_AVAILABLE = True
except ImportError:
    PXR_AVAILABLE = False
    Gf = None 
    PhysxSchema = None

ROS2_AVAILABLE = False

class ROS2_CONTROL_MODE(Enum):
    VEL = 1     # velocity control mode
    FORCE = 2   # force control mode
    TRANS = 3   # transform control mode

class ROS2ControlReceiver:
    """
    ROS2 Control Receiver
    
    for recieving velocity and force command
    """
    
    def __init__(self, robot_prim, name="ROS2ControlReceiver"):
        """
        initialize ROS2 Control Receiver
        
        Args:
            robot_prim: robot prim path
            name (str): receiver name
        """
        self._name = name
        self._robot_prim = robot_prim
        
        # configuration
        self._enable_ros2 = False
        
        self._ros2_control_mode = ROS2_CONTROL_MODE.VEL  # control mode
        self._ros2_vel_node = None
        self._ros2_force_node = None
        self._ros2_trans_node = None
        
        # command cache
        self.force_cmd = [0.0, 0.0, 0.0]
        self.torque_cmd = [0.0, 0.0, 0.0]
        self.linear_vel = [0.0, 0.0, 0.0]
        self.angular_vel = [0.0, 0.0, 0.0]
        self.translate_cmd = [0.0, 0.0, 0.0]
        self.orientation_cmd = [0.0, 0.0, 0.0, 1.0]
        self.last_command_time = time.time()
        self.command_timeout = 2.0
        self._update_count = 0
        
        # Physics API - using scenario.py created instance
        self._force_api = None
        self._scenario_force_api = None 
        
        print(f"[{self._name}] Initialized for robot prim")
        
    def initialize(self, enable_ros2=True, vel_topic="/oceansim/robot/vel_cmd", force_topic="/oceansim/robot/force_cmd", trans_topic="/oceansim/robot/trans_cmd"):
        """
        initialize reciever function
        
        Args:
            enable_ros2 (bool): whether using ros2
            vel_topic (str): topic name of vel
            force_topic (str): topic name of force(include torque)
        """
        self._enable_ros2 = enable_ros2
        self._vel_topic = vel_topic
        self._force_topic = force_topic
        self._trans_topic = trans_topic
        
        if not self._enable_ros2:
            print(f'[{self._name}] ROS2 disabled by configuration')
            return
        
        self._setup_subscriber()
        self._setup_physics()
        
        print(f'[{self._name}] Control Receiver Initialized:')
        print(f'[{self._name}] ROS2 Bridge: {self._enable_ros2}')
        if PXR_AVAILABLE and self._robot_prim:
            print(f'[{self._name}] Robot Prim: {self._robot_prim.GetPath()}')
        else:
            print(f'[{self._name}] Robot Prim: {self._robot_prim}')

    def set_scenario_force_api(self, scenario_force_api):
        """
        setting the force api
        """
        self._scenario_force_api = scenario_force_api
        
    def _setup_physics(self):
        """
        setting the physics control API(PXR)
        """
        if not PXR_AVAILABLE:
            print(f'[{self._name}] PXR not available, physics API disabled')
            return
            
        try:
            if self._scenario_force_api is not None:
                self._force_api = self._scenario_force_api
            else:
                if self._robot_prim.HasAPI(PhysxSchema.PhysxForceAPI):
                    self._force_api = PhysxSchema.PhysxForceAPI(self._robot_prim)
                else:
                    self._force_api = PhysxSchema.PhysxForceAPI.Apply(self._robot_prim)
                
        except Exception as e:
            print(f'[{self._name}] Physics API set failed: {e}')
    
    def _setup_subscriber(self):
        """
        setting the ROS2 subscriber
        """
        try:
            # import ROS2 module            
            # Initialize ROS2 context if not already done
            if not rclpy.ok():
                rclpy.init()
                print(f'[{self._name}] ROS2 context initialized')
            
            # Create velocity subscriber node
            node_name = f'oceansim_rob_velocity_control_{self._name.lower()}'.replace(' ', '_')
            self._ros2_vel_node = rclpy.create_node(node_name)
            self._ros2_vel_subscriber = self._ros2_vel_node.create_subscription(
                Twist,
                self._vel_topic,
                self._vel_callback,
                10
            )

            # Create force subscriber node
            node_name = f'oceansim_rob_force_control_{self._name.lower()}'.replace(' ', '_')
            self._ros2_force_node = rclpy.create_node(node_name)
            self._force_subscriber = self._ros2_force_node.create_subscription(
                Wrench,
                self._force_topic,
                self._force_callback,
                10
            )

            # Create trans subscriber node
            node_name = f'oceansim_rob_trans_control_{self._name.lower()}'.replace(' ', '_')
            self._ros2_trans_node = rclpy.create_node(node_name)
            self._trans_subscriber = self._ros2_trans_node.create_subscription(
                PoseStamped,
                self._trans_topic,
                self._trans_callback,
                10
            )
            
        except Exception as e:
            self._enable_ros2 = False

    def _setup_ros2_control_mode(self, ctrl_mode):
        if ctrl_mode == "velocity control":
            self._ros2_control_mode = ROS2_CONTROL_MODE.VEL
        elif ctrl_mode == "force control":
            self._ros2_control_mode = ROS2_CONTROL_MODE.FORCE
        elif ctrl_mode == "trans control":
            self._ros2_control_mode = ROS2_CONTROL_MODE.TRANS
    
    def _vel_callback(self, msg):
        """
        msg type: geometry_msgs/Twist
        
        include linear and angular velocity
        """
        print(f'[{self._name}] recieve ROS2 msg, type: {type(msg).__name__}, linear: {msg.linear}, angular: {msg.angular}')
        
        if not self._enable_ros2:
            print(f'[{self._name}] ROS2 is not enabled, ignore msg')
            return
        
        try:
            current_time = time.time()
            
            self.linear_vel = [msg.linear.x, msg.linear.y, msg.linear.z]
            self.angular_vel = [msg.angular.x, msg.angular.y, msg.angular.z]
            self.last_command_time = current_time
            
            # print(f'Received velocity - Linear: {self.linear_vel}, Angular: {self.angular_vel}')
            # self._update_receive_stats(current_time)
            
        except Exception as e:
            print(f'[{self._name}] Vel Receive Failed: {e}')
        
    def _force_callback(self, msg):
        """
        msg type: geometry_msgs/Wrench
        
        include force and torque
        """
        print(f'[{self._name}] recieve ROS2 msg, type: {type(msg).__name__}, force: {msg.force}, torque: {msg.torque}')

        if not self._enable_ros2:
            print(f'[{self._name}] ROS2 is not enabled, ignore msg')
            return

        try:
            current_time = time.time()

            self.force_cmd = [msg.force.x, msg.force.y, msg.force.z]  # force
            self.torque_cmd = [msg.torque.x, msg.torque.y, msg.torque.z]  # torque
            self.last_command_time = current_time

            # print(f'Received force - Force: {self.force_cmd}, Torque: {self.torque_cmd}')

        except Exception as e:
            print(f'[{self._name}] force Receive Failed: {e}')

    def _trans_callback(self, msg):
        """
        msg type: geometry_msgs/PoseStamped
        
        include translate and orintention
        """
        print(f'[{self._name}] recieve ROS2 msg, type: {type(msg).__name__}, position: {msg.pose.position}, orientation: {msg.pose.orientation}')

        if not self._enable_ros2:
            print(f'[{self._name}] ROS2 is not enabled, ignore msg')
            return

        try:
            current_time = time.time()

            self.translate_cmd = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
            self.orientation_cmd = [msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z]
            self.last_command_time = current_time

            # print(f'Received trans - Translate: {self.translate_cmd}, Orient: {self.orientation_cmd}')

        except Exception as e:
            print(f'[{self._name}] Trans Receive Failed: {e}')
    
    def update_control(self):
        """
        update control
        
        this function will be called in each simulation step. ( in scenario.update_scenario() )
        """
        if not self._enable_ros2 or not self._ros2_vel_node or not self._ros2_force_node:
            return
        
        try:
            if self._ros2_control_mode == ROS2_CONTROL_MODE.VEL: # velocity mode
                self._update_count += 1

                if self._update_count % 10 == 0: # need delay, otherwise the scene will be blocked
                    self._update_count = 0

                    rclpy.spin_once(self._ros2_vel_node, timeout_sec=0.0)
                    
                    rigid_prim = SingleRigidPrim(prim_path=get_prim_path(self._robot_prim))
                    rigid_prim.set_linear_velocity(np.array([0.0, 0.0, 0.0]))  # reset
                    rigid_prim.set_angular_velocity(np.array([0.0, 0.0, 0.0]))  # reset
                    rigid_prim.set_linear_velocity(np.array(self.linear_vel))
                    rigid_prim.set_angular_velocity(np.array(self.angular_vel))

            elif self._ros2_control_mode == ROS2_CONTROL_MODE.FORCE: # force mode
                # using PXR API to contorl
                if PXR_AVAILABLE:
                    
                    self._update_count += 1

                    if self._update_count % 10 == 0: # need delay, otherwise the scene will be blocked
                        self._update_count = 0

                        rclpy.spin_once(self._ros2_force_node, timeout_sec=0.0)

                        force_gf = Gf.Vec3f(float(self.force_cmd[0]), float(self.force_cmd[1]), float(self.force_cmd[2]))
                        torque_gf = Gf.Vec3f(float(self.torque_cmd[0]), float(self.torque_cmd[1]), float(self.torque_cmd[2]))
                    
                        if self._force_api:
                            try:
                                self._force_api.CreateForceAttr().Set(force_gf)
                                self._force_api.CreateTorqueAttr().Set(torque_gf)
                            except Exception as e:
                                print(f'[{self._name}] Force API Update Failed: {e}')

            elif self._ros2_control_mode == ROS2_CONTROL_MODE.TRANS:    # trans mode
                self._update_count += 1

                if self._update_count % 10 == 0:
                    self._update_count = 0
                    rclpy.spin_once(self._ros2_trans_node, timeout_sec=0.0)

                    if PXR_AVAILABLE and self._robot_prim:
                        self._robot_prim.GetAttribute('xformOp:translate').Set(Gf.Vec3f(*self.translate_cmd))
                        self._robot_prim.GetAttribute('xformOp:orient').Set(Gf.Quatd(*self.orientation_cmd))
                
        except Exception as e:
            print(f'[{self._name}] Control Update Failed: {e}')
    
    def close(self):
        # Clean up ROS2 resources
        if self._enable_ros2:
            if self._ros2_vel_node:
                self._ros2_vel_node.destroy_node()
            if self._ros2_force_node:
                self._ros2_force_node.destroy_node()
            if self._ros2_trans_node:
                self._ros2_trans_node.destroy_node()

        self._update_count = 0
        self.force_cmd = [0.0, 0.0, 0.0]
        self.torque_cmd = [0.0, 0.0, 0.0]
        self.linear_vel = [0.0, 0.0, 0.0]
        self.angular_vel = [0.0, 0.0, 0.0]
        self.translate_cmd = [0.0, 0.0, 0.0]
        self.orientation_cmd = [0.0, 0.0, 0.0, 1.0]

        rclpy.shutdown()

        print(f'[{self._name}] ROS2_Control_receiver closed.') 



class ROS2StatePublisher:
    def __init__(self, robot_prim, pub_frequency=1):
        self._robot_prim = robot_prim
        self._last_publish_time = 0.0
        self._pub_frequency = pub_frequency     # publish frequency, hz
        self._setup_ros2_publisher()

    def _setup_ros2_publisher(self):
        '''
        setup the publisher for state
        '''
        try:            
            # Initialize ROS2 context if not already done
            if not rclpy.ok():
                rclpy.init()
                print(f'[{self.__class__.__name__}] ROS2 context initialized')

            # Create publisher node
            node_name = f'oceansim_rob_state_pub'.replace(' ', '_')
            self._ros2_state_node = rclpy.create_node(node_name)

            self._vel_state_pub = self._ros2_state_node.create_publisher(Twist, '/oceansim/robot/state/vel', 10)
            self._trans_state_pub = self._ros2_state_node.create_publisher(PoseStamped, '/oceansim/robot/state/trans', 10)

        except Exception as e:
            print(f'[{self.__class__.__name__}] ROS2 state publisher setup failed: {e}')

    def _update_rob_state(self, rob):
        """
        update robot state info.
        """
        trans = [
            round(rob.GetAttribute('xformOp:translate').Get()[0], 2),           # translate x
            round(rob.GetAttribute('xformOp:translate').Get()[1], 2),           #           y
            round(rob.GetAttribute('xformOp:translate').Get()[2], 2),           #           z
            round(rob.GetAttribute('xformOp:orient').Get().real, 2),            # orient    w
            round(rob.GetAttribute('xformOp:orient').Get().imaginary[0], 2),    #           x
            round(rob.GetAttribute('xformOp:orient').Get().imaginary[1], 2),    #           y
            round(rob.GetAttribute('xformOp:orient').Get().imaginary[2], 2)     #           z
        ]
        velocity = [
            round(rob.GetAttribute('physics:velocity').Get()[0], 2),            # linear    x
            round(rob.GetAttribute('physics:velocity').Get()[1], 2),            #           y
            round(rob.GetAttribute('physics:velocity').Get()[2], 2),            #           z
            round(rob.GetAttribute('physics:angularVelocity').Get()[0], 2),     # angular   x
            round(rob.GetAttribute('physics:angularVelocity').Get()[1], 2),     #           y
            round(rob.GetAttribute('physics:angularVelocity').Get()[2], 2)      #           z
        ]

        self.state = {
            "trans": trans,
            "velocity": velocity
        }
        
        # print(state)

    def _publish_velocity(self, node, publisher, data):
        msg = Twist()
        msg.linear.x = data[0]
        msg.linear.y = data[1]
        msg.linear.z = data[2]
        msg.angular.x = data[3]
        msg.angular.y = data[4]
        msg.angular.z = data[5]
        publisher.publish(msg)

    def _publish_trans(self, node, publisher, data):
        msg = PoseStamped()
        msg.header.frame_id = 'world'
        msg.pose.position.x = data[0]
        msg.pose.position.y = data[1]
        msg.pose.position.z = data[2]
        msg.pose.orientation.w = data[3]
        msg.pose.orientation.x = data[4]
        msg.pose.orientation.y = data[5]
        msg.pose.orientation.z = data[6]
        msg.header.stamp = node.get_clock().now().to_msg()
        publisher.publish(msg)

    def publish_state(self):
        try:
            if not self._vel_state_pub or not self._trans_state_pub:
                return
            
            # frequency control
            current_time = time.time()
            if current_time - self._last_publish_time < (1.0 / self._pub_frequency):
                return
            
            self._last_publish_time = current_time

            self._update_rob_state(self._robot_prim)

            self._publish_velocity(self._ros2_state_node, self._vel_state_pub, self.state['velocity'])
            self._publish_trans(self._ros2_state_node, self._trans_state_pub, self.state['trans'])

            # print(f"{self.__class__.__name__} ROS2 state publish running")
            
        except Exception as e:
            print(f'[{self.__class__.__name__}] ROS2 state publish failed: {e}')

    def close(self):
        # Clean up ROS2 resources
        if self._ros2_state_node:
            self._ros2_state_node.destroy_node()

        self._last_publish_time = 0
        self.state = {}

        rclpy.shutdown()

        print(f'[{self.__class__.__name__}] closed.') 







