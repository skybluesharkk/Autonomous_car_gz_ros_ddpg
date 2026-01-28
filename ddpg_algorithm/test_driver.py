import rclpy
import torch
import numpy as np
import cv2
import time
import os
import argparse

from env_node import EnvNode
from ddpg_agent import DDPGAgent
from frame_stack import FrameStack

def main():
    parser = argparse.ArgumentParser(description='Test Driver and Video Recorder')
    parser.add_argument('--model', type=str, required=True, help='Path to the model checkpoint (.pth)')
    parser.add_argument('--output', type=str, default='driving_video.mp4', help='Output video filename')
    parser.add_argument('--max_steps', type=int, default=1000, help='Maximum steps to record')
    args = parser.parse_args()

    # ROS2 Init
    rclpy.init()
    
    # Environment & Agent
    env = EnvNode()
    agent = DDPGAgent()
    frame_stack = FrameStack(4)
    
    # Load Model
    print(f"Loading model from: {args.model}")
    if os.path.exists(args.model):
        checkpoint = torch.load(args.model)
        agent.Actor.load_state_dict(checkpoint['Actor'])
        agent.Actor.eval() # Set to evaluation mode
        print("Model loaded successfully!")
    else:
        print(f"Error: Model file {args.model} not found.")
        return

    # Video Writer Setup
    # EnvNode resizes to 64x64 for training, but let's check what we get.
    # We might want to capture the raw topic if possible, but EnvNode currently processes it.
    # EnvNode.latest_image is flattened/normalized float. We need to convert back to 0-255 uint8.
    
    # Wait for first observation to get dimensions
    print("Waiting for sensors...")
    env.reset()
    obs = env.get_observation()
    while obs['image'] is None:
        rclpy.spin_once(env)
        obs = env.get_observation()
    
    # Image is (64, 64, 3) float 0-1
    height, width, channels = 64, 64, 3
    # Upscale for better viewing? 64x64 is tiny. Let's resize x4 -> 256x256
    scale = 4
    video_w = width * scale
    video_h = height * scale
    
    fourcc = cv2.VideoWriter_fourcc(*'mp4v') # or 'XVID'
    out = cv2.VideoWriter(args.output, fourcc, 10.0, (video_w, video_h)) # 10fps estimated
    
    print(f"Recording started: {args.output}")
    
    state = frame_stack.reset(obs)
    
    steps = 0
    total_reward = 0
    
    try:
        while steps < args.max_steps:
            # Get Action (No Noise)
            # DDPGAgent.get_action adds noise by default unless we modify it or manually call Actor.
            # Let's manually call Actor for pure inference.
            
            # Prepare state
            state_img = torch.FloatTensor(state['image']).permute(0, 3, 1, 2).reshape(1, 12, 64, 64).to(agent.Actor.device)
            state_val = torch.FloatTensor(state['sensors']).flatten().unsqueeze(0).to(agent.Actor.device)
            
            with torch.no_grad():
                action_tensor = agent.Actor(state_img, state_val)
                action = action_tensor.cpu().numpy()[0]
                
            # Execute
            next_obs_raw, reward, done = env.step(action)
            total_reward += reward
            
            # Record Frame
            if next_obs_raw['image'] is not None:
                # Convert float 0-1 to uint8 0-255
                frame = (next_obs_raw['image'] * 255).astype(np.uint8)
                # RGB to BGR for OpenCV
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                # Upscale
                frame_large = cv2.resize(frame, (video_w, video_h), interpolation=cv2.INTER_NEAREST)
                out.write(frame_large)
                
                # Live Preview (Optional, helpful for debugging headless)
                # cv2.imshow('Driving', frame_large)
                # cv2.waitKey(1)
            
            # Stack state
            next_state = frame_stack.step(next_obs_raw)
            state = next_state
            
            steps += 1
            if steps % 100 == 0:
                print(f"Step {steps}/{args.max_steps}, Reward: {total_reward:.2f}")
                
            if done:
                print("Episode finished (Done).")
                break
                
    except KeyboardInterrupt:
        print("Interrupted by user.")
    finally:
        out.release()
        env.destroy_node()
        agent.destroy_node()
        rclpy.shutdown()
        print(f"Video saved to {args.output}")
        print(f"Total Reward: {total_reward:.2f}")

if __name__ == '__main__':
    main()
