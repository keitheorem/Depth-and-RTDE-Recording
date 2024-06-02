import pyrealsense2 as rs
import rtde_receive
import time
import threading
import os
import numpy as np
import cv2
from datetime import datetime
import keyboard  # External library to handle keyboard events

# Configure depth and color streams for the RealSense camera
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# Start the camera pipeline
pipeline.start(config)

# UR5 RTDE receiver setup
rtde_r = rtde_receive.RTDEReceiveInterface("192.168.0.100")  # Replace with your UR5's IP address

# Global variables for controlling the recording state
recording = False
current_episode = -1
episode_folder = None

# Function to capture depth image
def get_depth_image():
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    if not depth_frame:
        return None
    depth_image = np.asanyarray(depth_frame.get_data())
    return depth_image

# Function to get robot state
def get_robot_state():
    state = rtde_r.getActualTCPPose()
    return state

# Function to handle key presses
def handle_keys():
    global recording, current_episode, episode_folder

    while True:
        if keyboard.is_pressed('c'):
            if not recording:
                current_episode += 1
                episode_folder = str(current_episode)
                os.makedirs(os.path.join('depth', episode_folder), exist_ok=True)
                os.makedirs(os.path.join('state', episode_folder), exist_ok=True)
                recording = True
                print(f"Started recording episode {current_episode}")
            time.sleep(0.5)

        elif keyboard.is_pressed('p'):
            if recording:
                recording = False
                print(f"Paused recording episode {current_episode}")
            time.sleep(0.5)

        elif keyboard.is_pressed('q'):
            if recording:
                recording = False
            print("Quitting session...")
            break

        elif keyboard.is_pressed('d'):
            if current_episode >= 0:
                depth_path = os.path.join('depth', episode_folder)
                state_path = os.path.join('state', episode_folder)
                if os.path.exists(depth_path):
                    for file in os.listdir(depth_path):
                        os.remove(os.path.join(depth_path, file))
                    os.rmdir(depth_path)
                if os.path.exists(state_path):
                    for file in os.listdir(state_path):
                        os.remove(os.path.join(state_path, file))
                    os.rmdir(state_path)
                print(f"Discarded episode {current_episode}")
                current_episode -= 1
            time.sleep(0.5)

# Synchronization mechanism
def synchronized_capture(frequency=30):
    global recording, current_episode, episode_folder

    interval = 1.0 / frequency
    frame_counter = 0

    while True:
        start_time = time.time()

        if recording:
            # Get depth image and robot state
            depth_image = get_depth_image()
            robot_state = get_robot_state()

            if depth_image is not None and robot_state is not None:
                # Save depth image
                depth_filename = os.path.join('depth', episode_folder, f"depth_{frame_counter:06d}.png")
                cv2.imwrite(depth_filename, depth_image)

                # Save robot state
                state_filename = os.path.join('state', episode_folder, f"state_{frame_counter:06d}.txt")
                with open(state_filename, 'w') as f:
                    f.write(' '.join(map(str, robot_state)))

                frame_counter += 1
                print(f"Recorded frame {frame_counter} for episode {current_episode}")

        elapsed_time = time.time() - start_time
        time_to_sleep = interval - elapsed_time
        if time_to_sleep > 0:
            time.sleep(time_to_sleep)

# Main function to start the program
def main():
    global current_episode

    # Create directories if they don't exist
    os.makedirs('depth', exist_ok=True)
    os.makedirs('state', exist_ok=True)

    # Determine the highest existing episode number
    depth_episodes = [int(name) for name in os.listdir('depth') if os.path.isdir(os.path.join('depth', name))]
    state_episodes = [int(name) for name in os.listdir('state') if os.path.isdir(os.path.join('state', name))]

    if depth_episodes and state_episodes:
        current_episode = max(max(depth_episodes), max(state_episodes))

    # Start the synchronized capture in a separate thread
    capture_thread = threading.Thread(target=synchronized_capture)
    capture_thread.start()

    # Handle keyboard inputs in the main thread
    handle_keys()

    # Stop the pipeline when done
    pipeline.stop()
    print("Stopped synchronized capture.")

if __name__ == "__main__":
    main()
