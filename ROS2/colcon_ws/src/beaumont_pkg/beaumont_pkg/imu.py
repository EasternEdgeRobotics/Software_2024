import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import board
import adafruit_bno055

class ImuDataPublisher(Node):

    def __init__(self):
        super().__init__('imu_data_publisher')
        self.publisher_ = self.create_publisher(String, 'imu', 10)
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        #####
        self.i2c = board.I2C()
        self.sensor = adafruit_bno055.BNO055_I2C(self.i2c)
        self.last_val = 0xFFFF
        #####

    def get_temperature(self):
        result = self.sensor.temperature
        if abs(result - self.last_val) == 128:
            result = self.sensor.temperature
        if abs(result - self.last_val) == 128:
            return 0b00111111 & result
        self.last_val = result
        return result

    def timer_callback(self):
        msg = String()
        msg.data = "Temperature: {} degrees C,\n".format(self.sensor.temperature)
        msg.data = msg.data + "Accelerometer (m/s^2): {},\n".format(self.sensor.acceleration)
        msg.data = msg.data + "Magnetometer (microteslas): {},\n".format(self.sensor.magnetic)      
        msg.data = msg.data + "Gyroscope (rad/sec): {},\n".format(self.sensor.gyro)                 
        msg.data = msg.data + "Euler angle: {},\n".format(self.sensor.euler)               
        msg.data = msg.data + "Quaternion: {},\n".format(self.sensor.quaternion)              
        msg.data = msg.data + "Linear acceleration (m/s^2): {},\n".format(self.sensor.linear_acceleration)                
        msg.data = msg.data + "Gravity (m/s^2): {}".format(self.sensor.gravity) 
        self.publisher_.publish(msg)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    imu_data_publisher = ImuDataPublisher()

    rclpy.spin(imu_data_publisher)

    imu_data_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()