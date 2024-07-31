# Leg-KILO: Robust Kinematic-Inertial-Lidar Odometry for Dynamic Legged Robots

**Abstract** This paper presents  a robust multi-sensor fusion  framework, Leg-KILO (Kinematic-Inertial-Lidar Odometry). When lidar-based SLAM is applied to legged robots, high-dynamic motion (e.g., trot gait) introduces frequent foot impacts, leading to IMU degradation and lidar motion distortion. Direct use of IMU measurements can cause significant drift, especially in the z-axis direction. To address these limitations,  we tightly couple leg odometry, lidar odometry, and loop closure module based on graph optimization. For leg odometry, we propose a kinematic-inertial odometry using an on-manifold error-state Kalman filter, which incorporates the constraints from our proposed contact height detection to reduce height fluctuations. For lidar odometry,  we present an adaptive scan slicing and splicing method to alleviate the effects of high-dynamic motion. We further propose a robot-centric incremental mapping system that enhances map maintenance efficiency. Extensive experiments are conducted in both indoor and outdoor environments, showing that Leg-KILO has lower drift performance compared to other state-of-the-art lidar-based methods, especially during high-dynamic motion. To benefit the legged robot community, a lidar-inertial dataset containing leg kinematic data and the code  are released.

# dataset
Related datasets have been released in [link](https://github.com/ouguangjun/legkilo-dataset)

# video
The related video can be watched on [Youtube](https://youtu.be/6O74De5BLeQ). 

<a href="[https://youtu.be/HyLNq-98LRo](https://youtu.be/6O74De5BLeQ)" target="_blank"><img src="https://github.com/ouguangjun/Leg-KILO/blob/main/figure/youtube.png" 
alt="leg-kilo" width="300"  /></a>




# code
The code will be released once the paper accepted. 


<p align='center'>
    <img src="https://github.com/ouguangjun/kilo-dataset/blob/main/figure/map_dog.jpg" alt="drawing" width="600"/>
</p>


<p align='center'>
    <img src="https://github.com/ouguangjun/kilo-dataset/blob/main/figure/dog01.jpg" alt="drawing" width="300"/>
    <img src="https://github.com/ouguangjun/kilo-dataset/blob/main/figure/dog02.jpg" alt="drawing" width="300"/>
</p>

<p align='center'>
    <img src="https://github.com/ouguangjun/Leg-KILO/blob/main/figure/park.png" alt="drawing" width="900"/>
</p>
<p align='center'>
    <img src="https://github.com/ouguangjun/Leg-KILO/blob/main/figure/park02.png" alt="drawing" width="900"/>
</p>
<p align='center'>
    <img src="https://github.com/ouguangjun/Leg-KILO/blob/main/figure/corridor.png" alt="drawing" width="900"/>
</p>
<p align='center'>
    <img src="https://github.com/ouguangjun/Leg-KILO/blob/main/figure/corridor02.png" alt="drawing" width="900"/>
</p>
<p align='center'>
    <img src="https://github.com/ouguangjun/Leg-KILO/blob/main/figure/slope.png" alt="drawing" width="900"/>
</p>
<p align='center'>
    <img src="https://github.com/ouguangjun/Leg-KILO/blob/main/figure/slope02.png" alt="drawing" width="900"/>
</p>

# Acknowledgments

Thanks to [LIOSAM](https://github.com/TixiaoShan/LIO-SAM), [fast lio](https://github.com/hku-mars/FAST_LIO) and [A1-QP-MPC-Controller](https://github.com/ShuoYangRobotics/A1-QP-MPC-Controller) for sharing their works.
