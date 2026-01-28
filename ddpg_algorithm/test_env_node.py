import rclpy
from rclpy.node import Node
import sys
import os

# Add the current directory to path so we can import env_node
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)

try:
    from env_node import EnvNode
except ImportError as e:
    print(f"Failed to import EnvNode: {e}")
    sys.exit(1)

def main():
    print("Initializing ROS2...")
    rclpy.init()

    print("Creating EnvNode...")
    try:
        # Since EnvNode init declares parameters, we need to make sure it handles that without error.
        env = EnvNode()
        print("EnvNode created successfully.")
        
        print("Spinning node for a few iterations to check for startup errors...")
        for _ in range(5):
             rclpy.spin_once(env, timeout_sec=0.1)
        
        print("Spin test complete. No crashes during init.")
        
        # Check if subscriptions are set up
        print(f"Subscriptions: {len(env.subscriptions)}")
        print(f"Publishers: {len(env.publishers)}")
        print(f"Clients: {len(env.clients)}")
        
    except Exception as e:
        print(f"Error during EnvNode execution: {e}")
    finally:
        print("Shutting down...")
        rclpy.shutdown()

if __name__ == "__main__":
    main()
