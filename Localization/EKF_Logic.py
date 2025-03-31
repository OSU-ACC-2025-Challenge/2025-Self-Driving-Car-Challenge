#!/usr/bin/env python3

import numpy as np
import time
import math
from pal.qcar import QCar  # Using pal library as required

class EKFLocalization:
    def __init__(self):
        # Initialize QCar interface
        self.qcar = QCar()
        
        # State vector: [x, y, theta, v, omega]
        # x, y: position in meters
        # theta: heading in radians
        # v: linear velocity in m/s
        # omega: angular velocity in rad/s
        self.state = np.zeros(5)
        
        # Initial position uncertainty (large for unknown starting position)
        # Diagonal elements represent variance of each state variable
        self.covariance = np.diag([10.0, 10.0, 0.1, 0.1, 0.1])
        
        # Process noise (uncertainty in motion model)
        self.Q = np.diag([0.01, 0.01, 0.01, 0.1, 0.1])
        
        # Measurement noise for different sensors
        self.R_gps = np.diag([0.5, 0.5])  # GPS position noise (x, y)
        self.R_imu = np.diag([0.01, 0.1])  # IMU noise (theta, omega)
        self.R_encoder = np.diag([0.1])  # Encoder noise (v)
        
        # Last update time for dt calculation
        self.last_time = time.time()
        
        print("EKF Localization initialized")
    
    def predict(self, dt, control_input=None):
        """
        Prediction step of EKF.
        dt: time step in seconds
        control_input: control inputs if available, else use current state velocity
        """
        # Simple bicycle model for prediction
        # If control_input is None, use current velocity estimates in state
        v = self.state[3]
        omega = self.state[4]
        
        # Extract current state
        x = self.state[0]
        y = self.state[1]
        theta = self.state[2]
        
        # Predict next state using motion model
        # x' = x + v * cos(theta) * dt
        # y' = y + v * sin(theta) * dt
        # theta' = theta + omega * dt
        # v' = v (assume constant velocity in prediction)
        # omega' = omega (assume constant angular velocity in prediction)
        self.state[0] = x + v * np.cos(theta) * dt
        self.state[1] = y + v * np.sin(theta) * dt
        self.state[2] = theta + omega * dt
        # Velocity and angular velocity remain unchanged in prediction
        
        # Compute Jacobian of motion model
        F = np.eye(5)
        F[0, 2] = -v * np.sin(theta) * dt
        F[0, 3] = np.cos(theta) * dt
        F[1, 2] = v * np.cos(theta) * dt
        F[1, 3] = np.sin(theta) * dt
        F[2, 4] = dt
        
        # Update covariance
        self.covariance = F @ self.covariance @ F.T + self.Q * dt
        
        return self.state, self.covariance
    
    def update_gps(self, gps_measurement):
        """
        Update step using GPS measurement
        gps_measurement: [x, y] position from GPS
        """
        # Measurement model for GPS is direct observation of position
        H = np.zeros((2, 5))
        H[0, 0] = 1.0  # x position 
        H[1, 1] = 1.0  # y position
        
        # Expected measurement based on current state
        expected_measurement = np.array([self.state[0], self.state[1]])
        
        # Innovation: difference between actual and expected measurement
        innovation = gps_measurement - expected_measurement
        
        # Innovation covariance
        S = H @ self.covariance @ H.T + self.R_gps
        
        # Kalman gain
        K = self.covariance @ H.T @ np.linalg.inv(S)
        
        # Update state
        self.state = self.state + K @ innovation
        
        # Update covariance
        self.covariance = (np.eye(5) - K @ H) @ self.covariance
        
        return self.state, self.covariance
    
    def update_imu(self, imu_measurement):
        """
        Update step using IMU measurement
        imu_measurement: [theta, omega] orientation and angular velocity from IMU
        """
        # Measurement model for IMU
        H = np.zeros((2, 5))
        H[0, 2] = 1.0  # theta
        H[1, 4] = 1.0  # omega
        
        # Expected measurement based on current state
        expected_measurement = np.array([self.state[2], self.state[4]])
        
        # Innovation
        innovation = imu_measurement - expected_measurement
        
        # Ensure angle difference is within -pi to pi
        innovation[0] = (innovation[0] + np.pi) % (2 * np.pi) - np.pi
        
        # Innovation covariance
        S = H @ self.covariance @ H.T + self.R_imu
        
        # Kalman gain
        K = self.covariance @ H.T @ np.linalg.inv(S)
        
        # Update state
        self.state = self.state + K @ innovation
        
        # Update covariance
        self.covariance = (np.eye(5) - K @ H) @ self.covariance
        
        return self.state, self.covariance
    
    def update_encoder(self, velocity_measurement):
        """
        Update step using wheel encoder measurement
        velocity_measurement: linear velocity estimate from wheel encoders
        """
        # Measurement model for encoder
        H = np.zeros((1, 5))
        H[0, 3] = 1.0  # v
        
        # Expected measurement based on current state
        expected_measurement = np.array([self.state[3]])
        
        # Innovation
        innovation = velocity_measurement - expected_measurement
        
        # Innovation covariance
        S = H @ self.covariance @ H.T + self.R_encoder
        
        # Kalman gain
        K = self.covariance @ H.T @ np.linalg.inv(S)
        
        # Update state
        self.state = self.state + K @ innovation
        
        # Update covariance
        self.covariance = (np.eye(5) - K @ H) @ self.covariance
        
        return self.state, self.covariance
    
    def run_ekf_iteration(self):
        """
        Run one iteration of the EKF by:
        1. Reading sensor data from QCar
        2. Performing prediction step
        3. Performing update steps with available sensor data
        """
        # Calculate time step
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        # Read sensor data from QCar (adapt based on actual API)
        # This is where you would interface with the pal library
        try:
            # Read GPS-like position data (adapt to actual QCar API)
            gps_data = self.read_gps_data()
            
            # Read IMU data
            imu_data = self.read_imu_data()
            
            # Read encoder data 
            encoder_data = self.read_encoder_data()
            
            # Prediction step
            self.predict(dt)
            
            # Update steps with available sensor data
            if gps_data is not None:
                self.update_gps(gps_data)
            
            if imu_data is not None:
                self.update_imu(imu_data)
            
            if encoder_data is not None:
                self.update_encoder(encoder_data)
            
            return True
            
        except Exception as e:
            print(f"Error in EKF iteration: {e}")
            return False
    
    def read_gps_data(self):
        """
        Read GPS-like position data from QCar
        Returns [x, y] position or None if not available
        """
        # This is a placeholder - adapt to actual QCar API
        # In the virtual environment, you might have direct access to position
        # For example: qcar.get_position() might return the position
        try:
            # This is pseudocode - replace with actual QCar API calls
            sensors = self.qcar.read_sensors()
            if hasattr(sensors, 'gps'):
                return np.array([sensors.gps.x, sensors.gps.y])
            return None
        except:
            return None
    
    def read_imu_data(self):
        """
        Read IMU data from QCar
        Returns [theta, omega] or None if not available
        """
        try:
            # This is pseudocode - replace with actual QCar API calls
            sensors = self.qcar.read_sensors()
            if hasattr(sensors, 'imu'):
                # Extract orientation (theta) and angular velocity (omega)
                theta = sensors.imu.orientation_z  # Assuming this gives yaw
                omega = sensors.imu.angular_velocity_z
                return np.array([theta, omega])
            return None
        except:
            return None
    
    def read_encoder_data(self):
        """
        Read encoder data from QCar
        Returns [velocity] or None if not available
        """
        try:
            # This is pseudocode - replace with actual QCar API calls
            sensors = self.qcar.read_sensors()
            if hasattr(sensors, 'encoders'):
                # Calculate velocity from wheel encoders
                velocity = sensors.encoders.velocity  # Assuming this exists
                return np.array([velocity])
            return None
        except:
            return None
    
    def get_position(self):
        """
        Returns the current estimated position and orientation
        """
        return {
            'x': self.state[0],
            'y': self.state[1],
            'theta': self.state[2],
            'uncertainty_x': self.covariance[0, 0],
            'uncertainty_y': self.covariance[1, 1],
            'uncertainty_theta': self.covariance[2, 2]
        }


# Example usage
if __name__ == "__main__":
    # Initialize EKF
    ekf = EKFLocalization()
    
    try:
        # Main loop
        while True:
            # Run one iteration of EKF
            success = ekf.run_ekf_iteration()
            
            if success:
                # Get current position estimate
                position = ekf.get_position()
                print(f"Position: ({position['x']:.2f}, {position['y']:.2f}), "
                      f"Heading: {position['theta']:.2f} rad, "
                      f"Uncertainties: ({position['uncertainty_x']:.2f}, "
                      f"{position['uncertainty_y']:.2f}, {position['uncertainty_theta']:.2f})")
            
            # Sleep to control update rate (10Hz)
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("EKF localization stopped by user")