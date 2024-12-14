
import rclpy
from rclpy.node import Node
import serial
import time
from std_msgs.msg import Bool
from std_msgs.msg import Int32

class ArduinoController(Node):
    def __init__(self):
        super().__init__('arduino_controller')
        
        # Set up the serial connection to the Arduino
        self.arduino = serial.Serial('/dev/ttyACM1', 9600)
        time.sleep(2)  # Wait for the Arduino to reset

        # Create a publisher to publish pin state messages
        self.publisher = self.create_publisher(Bool, 'pin_state', 10)

        # Create a subscription to listen for pin control commands
        self.subscription = self.create_subscription(
            Int32,
            'pin_control',
            self.pin_control_callback,
            10
        )

        self.state = 0;

    def send_pin_state(self, pin, state):
        """Send the pin number and state (low/high) to the Arduino."""
        if state not in [0, 1]:
            self.get_logger().warn("State must be 0 (low) or 1 (high).")
            return
        # Send the pin and state as two bytes
        self.arduino.write(bytes([pin, state]))

   
    def pin_control_callback(self, msg):
        """Callback to handle pin control messages."""
        pin = 13  # Example: you can expand this to other pins if needed
        self.send_pin_state(pin, msg.data)
        self.get_logger().info(f'Received pin control message: {msg.data}')


def main(args=None):
    rclpy.init(args=args)

    arduino_controller = ArduinoController()

    try:
        rclpy.spin(arduino_controller)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        arduino_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

