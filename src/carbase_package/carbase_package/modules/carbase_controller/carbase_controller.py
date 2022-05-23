from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool


class CarbaseController(Node):
    def __init__(self, name = None, publisher_topic = None, triangle_publisher_topic = None,circle_publisher_topic = None,cross_publisher_topic = None,square_publisher_topic = None,Dup_publisher_topic = None,Ddow_publisher_topic = None):
        self.name = name if name is not None else "CarbaseController"
        self.publisher_topic = publisher_topic if publisher_topic is not None else "carbase_cmd"
        self.triangle_publisher_topic = triangle_publisher_topic if triangle_publisher_topic is not None else "triangle_cmd"
        self.circle_publisher_topic = circle_publisher_topic if circle_publisher_topic is not None else "circle_cmd"
        self.cross_publisher_topic = cross_publisher_topic if cross_publisher_topic is not None else "cross_cmd"
        self.square_publisher_topic = square_publisher_topic if square_publisher_topic is not None else "square_cmd"
        self.Dup_publisher_topic = Dup_publisher_topic if Dup_publisher_topic is not None else "Dup_cmd"
        self.Ddow_publisher_topic = Ddow_publisher_topic if Ddow_publisher_topic is not None else "Ddow_cmd"
        super().__init__(self.name)
        
        # Publisher
        self.publisher = self.create_publisher(Twist, self.publisher_topic, 10)
        self.triangle_publisher = self.create_publisher(Bool, self.triangle_publisher_topic, 10)
        self.circle_publisher = self.create_publisher(Bool, self.circle_publisher_topic, 10)
        self.cross_publisher = self.create_publisher(Bool, self.cross_publisher_topic, 10)
        self.square_publisher = self.create_publisher(Bool, self.square_publisher_topic, 10)
        self.Dup_publisher = self.create_publisher(Bool, self.Dup_publisher_topic, 10)
        self.Ddow_publisher = self.create_publisher(Bool, self.Ddow_publisher_topic, 10)
        self.carbase_msg = Twist()
        self.min, self.max = -10.0, 10.0
        self.target_y, self.step_y = 0.0, 0.25  # Horizontal
        self.target_x, self.step_x = 0.0, 0.25  # Vertical
        self.target_z, self.step_z = 0.0, 0.25  # Rotation 
        self.pre_y_speed = 0.0 
        self.pre_x_speed = 0.0 
        #self.servo_angle = 0  # clamp servo rotation in radians   
        self.triangle = Bool()
        self.triangle.data = False
        self.circle = Bool()
        self.circle.data = False
        self.cross = Bool()
        self.cross.data = False
        self.square = Bool()
        self.square.data = False
        self.Dup = Bool()
        self.Dup.data = False
        self.Ddow = Bool()
        self.Ddow.data = False
        # Subscriber
        self.subscription = self.create_subscription(Joy, "joy", self.joyCallback, 10)

        self.get_logger().info(f"{self.name} created successfully")

    def constrain(self, x, min, max):
        if x < min:
            return min
        elif x > max:
            return max

        return x

    def joyCallback(self, msg):
        # Horizontal
        if msg.axes[0] != 0.0:
            if (self.pre_y_speed - self.target_y) > 7 or (self.pre_y_speed - self.target_y) < -7:
                self.target_y = self.constrain(self.target_y + self.step_y if msg.axes[0] > 0.0 else self.target_y - self.step_y, self.pre_y_speed + self.step_y, self.pre_y_speed - self.step_y)
                self.pre_y_speed = self.target_y
            else:
                self.target_y = self.constrain(self.target_y + self.step_y if msg.axes[0] > 0.0 else self.target_y - self.step_y, abs(msg.axes[0]) * -10.0, abs(msg.axes[0]) * 10.0)
                self.pre_y_speed = self.target_y
        else:
            if self.target_y < 0.0:
                self.target_y = self.constrain(self.target_y + self.step_y, self.min, 0.0)
            elif self.target_y > 0.0:
                self.target_y = self.constrain(self.target_y - self.step_y, 0.0, self.max)

        # Vertical
        if msg.axes[1] != 0.0:
            if (self.pre_x_speed - self.target_x) > 7 or (self.pre_x_speed - self.target_x) < -7:
                self.target_x = self.constrain(self.target_x + self.step_x if msg.axes[1] > 0.0 else self.target_x - self.step_x, self.pre_x_speed + self.step_x, self.pre_x_speed - self.step_x)
                self.pre_x_speed = self.target_x
            else:
                self.target_x = self.constrain(self.target_x + self.step_x if msg.axes[1] > 0.0 else self.target_x - self.step_x, abs(msg.axes[1]) * -10.0, abs(msg.axes[1]) * 10.0)
                self.pre_x_speed = self.target_x
        else:
            if self.target_x < 0.0:
                self.target_x = self.constrain(self.target_x + self.step_x, self.min, 0.0)
            elif self.target_x > 0.0:
                self.target_x = self.constrain(self.target_x - self.step_x, 0.0, self.max)

        # Rotation
        if msg.axes[2] != 0.0:
            self.target_z = self.constrain(self.target_z + self.step_z if msg.axes[2] > 0.0 else self.target_z - self.step_z, abs(msg.axes[2]) * -5.0, abs(msg.axes[2]) * 5.0)
        else:
            if self.target_z < 0.0:
                self.target_z = self.constrain(self.target_z + self.step_z, self.min, 0.0)
            elif self.target_z > 0.0:
                self.target_z = self.constrain(self.target_z - self.step_z, 0.0, self.max)
       
        self.carbase_msg.linear.y = self.target_y
        self.carbase_msg.linear.x = self.target_x
        self.carbase_msg.angular.z = self.target_z

        self.publisher.publish(self.carbase_msg)

        if msg.buttons[3] != self.triangle.data:
            self.triangle.data = bool(msg.buttons[3])
            self.get_logger().info('triangle: %r' %(self.triangle.data))
            self.triangle_publisher.publish(self.triangle)

        if msg.buttons[2] != self.circle.data:
            self.circle.data = bool(msg.buttons[2])
            self.get_logger().info('circle: %r' %(self.circle.data))
            self.circle_publisher.publish(self.circle)

        if msg.buttons[1] != self.cross.data:
            self.cross.data = bool(msg.buttons[1])
            self.get_logger().info('cross: %r' %(self.cross.data))
            self.cross_publisher.publish(self.cross)

        if msg.buttons[0] != self.square.data:
            self.square.data = bool(msg.buttons[0])
            self.get_logger().info('cross: %r' %(self.square.data))
            self.square_publisher.publish(self.square)

        if msg.axes[7] == 1:
            self.Dup.data = True
            self.get_logger().info('Dup: %r' %(self.Dup.data))
            self.Dup_publisher.publish(self.Dup)
        else:
            self.Dup.data = False
            self.get_logger().info('Dup: %r' %(self.Dup.data))
            self.Dup_publisher.publish(self.Dup)

        if msg.axes[7] == -1:
            self.Ddow.data = True
            self.get_logger().info('Ddow: %r' %(self.Ddow.data))
            self.Ddow_publisher.publish(self.Ddow)
        else:
            self.Ddow.data = False
            self.get_logger().info('Ddow: %r' %(self.Ddow.data))
            self.Ddow_publisher.publish(self.Ddow)


