# LOCALIZATION

**ALGORITHM:** EKF (Extended Kalman Filter). 
[NOTE: Could possibly implement SLAM if EKF doesn't hold up/isn't robust]

**INPUT:** 
1. Position Coordinates [x, y],  
2. IMU Data: [Orientation/Heading(theta), Angualr Velocity(omega)]
3. Velocity of QCar2 [Using Wheel Encoder] 
[NOTE: Check if QCar2 has wheel encoders]

**STATUS:** 
1. Main logic added (EKF_Logic.py) with placeholders for above input data with relevant datatype

**TBD:**
1. Integrate QCar2 input sensor data from *pal* library.
2. Integrate with ROS2 modules/nodes.
3. Test

*PERSONAL NOTE:* SLAM-based algo. might take longer to implement (Lidar pre-processing + Point-cloud generation)