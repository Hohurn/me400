import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TurtleBotController(Node):
    def __init__(self):
        super().__init__('turtlebot_controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cap = cv2.VideoCapture(0)
    
    def process_frame(self, frame):
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Convert to grayscale
        height, width = img.shape
        
        # Compute darkness sum along the y-axis
        darkness_profile = np.sum(255 - img, axis=0)
        
        # Threshold for detecting dark areas
        darkness_threshold = np.mean(darkness_profile) + 1.5 * np.std(darkness_profile)
        pole_positions = np.where(darkness_profile > darkness_threshold)[0]
        
        return pole_positions, width
    
    def control_turtlebot(self):
        twist = Twist()
        
        while self.cap.isOpened():
            ret, frame = self.cap.read()
            if not ret:
                break
            
            pole_positions, width = self.process_frame(frame)
            
            if len(pole_positions) > 0:
                pole_0 = pole_positions[0]
                pole_1 = pole_positions[-1]
                pole_mid = (pole_0 + pole_1) / 2
                
                # Compute steering percentage (-100 to 100)
                steering_offset = (pole_mid - width / 2) / (width / 2)
                
                # Define control rules
                twist.linear.x = 0.0  # Move forward with constant speed
                twist.angular.z = -steering_offset * 0.5  # Turn based on pole position
                
                print(f"Linear: {twist.linear.x:.2f}, Angular: {twist.angular.z:.2f}")
            else:
                print("No pole detected, stopping")
                twist.linear.x = 0.0
                twist.angular.z = 0.0  # Stop if no pole is detected
            
            # Publish velocity command
            self.publisher_.publish(twist)

            # Display the frame
            # cv2.imshow('TurtleBot Camera', frame)
            
            # Exit on 'q' key press
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        self.cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    controller = TurtleBotController()
    controller.control_turtlebot()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
