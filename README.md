# Robust Continuous Sliding Mode Control for PMSM Manipulator Trajectory

## Introduction

In the paper titled **"Robust Continuous Sliding Mode Control for Manipulator PMSM Trajectory"**, the research focuses on the application of Permanent Magnet Synchronous Motors (PMSM) in commercial manipulators. PMSMs are known for their high accuracy, high power density, and fast response. However, in real-world operations, the PMSM systems in manipulators often face time-varying uncertain disturbances.

Conventional linear control methods like PID struggle to achieve good performance in the presence of such disturbances, leading to vibrations and inaccuracies. Thus, there is a need for robust nonlinear control methods. One such method that has garnered attention is the Sliding Mode Control (SMC) algorithm. However, the chattering phenomenon remains a significant issue with SMC.

To address chattering, researchers have proposed various methods, such as selecting appropriate gains or combining SMC with other techniques. Recent advancements in sliding mode control include composite nonlinear control, integral sliding mode control, and terminal sliding mode control.

This paper introduces a new method called **Robust Continuous Sliding Mode Control (RCSMC)** for the trajectory tracking of PMSM in manipulators subjected to time-varying uncertain disturbances. The RCSMC method comprises the Terminal Continuous Sliding Mode Control (TCSMC) algorithm and an extended state observer. The TCSMC algorithm is designed to reduce chattering and enhance stabilization speed, though it requires large control gains. Therefore, the extended state observer is used to estimate disturbances and improve control performance.

## Features of RCSMC

1. **High Precision**: Achieves high accuracy in trajectory tracking for PMSM manipulators.
2. **Robustness**: Effectively handles time-varying uncertain disturbances.
3. **Reduced Chattering**: Utilizes TCSMC to minimize the chattering phenomenon.
4. **Improved Stabilization**: Enhances the speed of stabilization.
5. **Disturbance Estimation**: Uses an extended state observer to estimate and compensate for disturbances.

## Implementation

The RCSMC method integrates advanced control algorithms and state observation techniques to ensure optimal performance of PMSM manipulators in dynamic and uncertain environments. This approach promises significant improvements in precision and robustness compared to conventional control methods.
![image](https://github.com/Alfanfuad/SM-PI-Control-Strategy-of-Electric-Motor-Pump-for-Pure-Electric-Construction-Machinery/assets/71118100/7ec2b787-4d5b-401a-9798-3bcf43cf79d9)
