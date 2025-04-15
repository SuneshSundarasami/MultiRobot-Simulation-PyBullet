import unittest
import pybullet as p
from unittest.mock import patch, MagicMock
import sys
import os

project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, project_root)
from robile_pybullet.multi_robot_sim import MultiRobileController

class TestMultiRobileController(unittest.TestCase):

    @patch('pybullet.connect', return_value=1)
    @patch('pybullet.setAdditionalSearchPath')
    @patch('pybullet.setGravity')
    @patch('pybullet.loadURDF', return_value=1)
    @patch('pybullet.setJointMotorControl2')
    def test_robot_spawning(self, mock_setJointMotorControl2, mock_loadURDF, mock_setGravity, mock_setAdditionalSearchPath, mock_connect):
        print("\nStarting test_robot_spawning...")
        
        urdf_path = "dummy_path.urdf"
        num_followers = 4
        formation_config = [(-1.0, -1.0), (-1.0, 1.0), (-2.0, -2.0), (-2.0, 2.0)]
        
        print("Initializing MultiRobileController...")
        controller = MultiRobileController(urdf_path, num_followers, formation_config, connection_mode=p.DIRECT)
        
        print("Checking if loadURDF was called correct number of times...")
        expected_calls = num_followers + 2  # num_followers + leader + ground plane
        self.assertEqual(mock_loadURDF.call_count, expected_calls)
        print(f"loadURDF called {mock_loadURDF.call_count} times, expected {expected_calls}")
        
        print("Checking if correct number of follower IDs were stored...")
        self.assertEqual(len(controller.follower_ids), num_followers)
        print(f"Number of follower IDs: {len(controller.follower_ids)}, expected {num_followers}")
        
        print("Checking if leader ID is not None...")
        self.assertIsNotNone(controller.leader_id)
        print(f"Leader ID: {controller.leader_id}")
        
        print("Test completed successfully!")

    @patch('pybullet.connect', return_value=1)
    @patch('pybullet.setAdditionalSearchPath')
    @patch('pybullet.setGravity')
    @patch('pybullet.loadURDF', return_value=1)
    @patch('pybullet.setJointMotorControl2')
    @patch('pybullet.getBasePositionAndOrientation', return_value=((0, 0, 0), (0, 0, 0, 1)))
    @patch('pybullet.getEulerFromQuaternion', return_value=(0, 0, 0))
    @patch('pybullet.stepSimulation')
    def test_maintain_distance(self, mock_step, mock_euler, mock_pos, mock_joint, mock_load, mock_gravity, mock_path, mock_connect):
        urdf_path = "dummy_path.urdf"
        num_followers = 4
        formation_config = [(-1.0, -1.0), (-1.0, 1.0), (-2.0, -2.0), (-2.0, 2.0)]
        
        print("\nInitializing MultiRobileController for maintain distance test...")
        controller = MultiRobileController(urdf_path, num_followers, formation_config)

        print("Simulating movement for the leader and followers...")
        # Simulate movement
        for iteration in range(10):
            print(f"\nIteration {iteration + 1}: Moving leader omnidirectionally...")
            controller.move_omnidirectional(0.5, 0, 0, controller.leader_id)
            controller.update_followers()
            controller.run_simulation()
        
        
        print("\nChecking call counts for verification...")
        # Check if update_followers was called
        self.assertEqual(mock_pos.call_count, (num_followers + 1) * 10) # Leader + followers
        print(f"Base position calls: {mock_pos.call_count} (Expected: {(num_followers + 1) * 10})")
        
        self.assertEqual(mock_euler.call_count, (num_followers + 1) * 10) # Leader + followers
        print(f"Euler angle calls: {mock_euler.call_count} (Expected: {(num_followers + 1) * 10})")
        
        self.assertEqual(mock_step.call_count, 10)  
        print(f"Step simulation calls: {mock_step.call_count} (Expected: 10)")  

    def tearDown(self):
        if p.isConnected():
            p.disconnect()
            print("Disconnected from PyBullet.")

if __name__ == '__main__':
    unittest.main(verbosity=2)
