import pybullet as p
import pybullet_data
import time
import json
from pathlib import Path

# Path to the JSON file
json_path = Path(__file__).parent.parent / "path.json"

# Load JSON data
with open(json_path, 'r') as f:
    data = json.load(f)

# Initialize PyBullet
def main():
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    # Disable physics simulation
    p.setRealTimeSimulation(0)

    # Load the robot from the URDF file
    urdf_path = data["urdf_path"]
    robot_id = p.loadURDF(urdf_path, [0, 0, 0])

    # Draw spheres at the specified locations
    spheres = data["spheres"]
    for sphere in spheres:
        p.createCollisionShape(p.GEOM_SPHERE, radius=sphere["radius"])
        p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=-1,
            baseVisualShapeIndex=p.createVisualShape(
                p.GEOM_SPHERE, radius=sphere["radius"], rgbaColor=[1, 0, 0, 1]
            ),
            basePosition=sphere["center"],
        )

    # Draw a copy of the robot in green at the goal location
    goal = data["goal"]  # Goal joint positions
    num_joints = len(goal)
    
    goal_robot_id = p.loadURDF(urdf_path, [0, 0, 0], useFixedBase=True)
    for joint_idx in range(num_joints):
        p.resetJointState(goal_robot_id, joint_idx, goal[joint_idx])
    
    # Change color of the goal robot
    for joint_idx in range(p.getNumJoints(goal_robot_id)):
        p.changeVisualShape(goal_robot_id, joint_idx, rgbaColor=[0, 1, 0, 1])

    # Path to follow
    path = data["path"]

    # Step the robot's position forward
    for target_joint_positions in path:
        for joint_idx in range(num_joints):
            p.resetJointState(robot_id, joint_idx, target_joint_positions[joint_idx])
        time.sleep(0.1)  # Optional for visualization

    # Keep the simulation open
    while p.isConnected():
        time.sleep(1)

if __name__ == "__main__":
    main()
