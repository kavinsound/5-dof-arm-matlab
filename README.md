# 5-DOF Robotic Manipulator: Multibody Dynamics & Trajectory Control

An end-to-end simulation, kinematic analysis, and control system design for a 5-Degree-of-Freedom (5-DOF) serial open-chain robotic manipulator, developed in **MATLAB & Simscape Multibody**. 

This project bypasses traditional geometric abstractions to model true rigid-body link dynamics, utilizing hybrid numerical/analytical Inverse Kinematics (IK) for Cartesian positioning and independent parallel PID controllers for joint-space trajectory tracking.

---

## 🚀 Key Technical Highlights
* **High-Fidelity Physics:** Modeled physical link mass, inertia tensors, and joint friction profiles directly in Simscape Multibody, moving beyond pure kinematic animations.
* **Control Architecture:** Designed and tuned 5 independent parallel PID controllers to handle nonlinear gravitational and inertial cross-coupling torques during transient trajectories.
* **Coordinate-Free Kinematics:** Bypassed restrictive Denavit-Hartenberg (D-H) frame constraints by leveraging Simscape’s free 3D spatial transformations, mapping joint frames explicitly to physical CAD intersections.

---

## 🛠️ System Architecture & Mechanical Design

The manipulator consists of a 5-DOF open-chain configuration. All 5 joints are revolute, each rotating about its local Z-axis. 

### Joint Configuration & Work Envelope Limits
| Joint | Type | Axis of Rotation | Motion Range | Purpose / Function |
| :---: | :---: | :---: | :---: | :--- |
| **J1** | Revolute | Local Z | $\pm180^\circ$ | Base Rotation (Yaw) |
| **J2** | Revolute | Local Z | $+20^\circ \text{ to } +350^\circ$ | Shoulder Link (Pitch) |
| **J3** | Revolute | Local Z | $\pm120^\circ$ | Elbow Link (Pitch) |
| **J4** | Revolute | Local Z | $\pm180^\circ$ | Wrist Pitch / Orientation |
| **J5** | Revolute | Local Z | $-45^\circ \text{ to } +225^\circ$ | Wrist Roll / End-Effector Twist |

<img src="pics/mechanical_arm-axis.png" alt="Simscape Local Coordinate Frames" width="600">

*Figure 1: Local coordinate frames at joint intersections inside Simscape Multibody, ensuring precise spatial alignment.*

---

## 🧠 Kinematics & Control Loop

The system operates on a closed-loop architecture where desired Cartesian trajectories are mapped to joint space, and executed via localized feedback control.

```mermaid
graph LR
    %% Style definitions
    classDef default fill:#1f2937,stroke:#4b5563,stroke-width:2px,color:#f9fafb;
    classDef highlight fill:#1e3a8a,stroke:#3b82f6,stroke-width:2px,color:#eff6ff;

    A([Desired Cartesian<br/>Trajectory<br/><b>X, Y, Z</b>]) --> B[IK Solver]
    B -->|Desired Joint Angles<br/><i>θ<sub>i</sub></i>| C[Parallel PID Loops]
    C -->|Torque Actuation| D[Simscape Multibody]
    D --- E{Feedback}
    E -.->|Joint States| C

    class A,D highlight;
```

1. **Inverse Kinematics (IK):** A hybrid numerical-analytical solver resolves the non-linear transcendental equations mapping the end-effector's Cartesian targets $(X,Y,Z)$ into required joint space vectors $\vec{\theta} = [\theta_1, \theta_2, \theta_3, \theta_4, \theta_5]^T$.
2. **Trajectory Tracking:** The resolved angles feed into the parallel PID control loops as time-varying setpoints.

---

## 📊 Simulation & Verification

### Transient Performance (PID Tuning)
To overcome gravity loading and multi-link dynamic coupling, the joint actuators were iteratively tuned. Below is the step-response comparison showcasing the elimination of steady-state error and minimization of transient overshoot.

| Before Tuning (Default/Unoptimized) | After Tuning (Optimized Parallel PID) |
|---|---|
<table id="pid-comparison">
  <tr>
    <td align="center">
      <b>Before Tuning (Default/Unoptimized)</b><br/>
      <img src="pics/pid_tuning_before.png" alt="PID Before" width="380"/>
      <br/><i>High settling time, steady-state error under gravity.</i>
    </td>
    <td align="center">
      <b>After Tuning (Optimized Parallel PID)</b><br/>
      <img src="pics/pid_tuning_after.png" alt="PID After" width="380"/>
      <br/><i>Critically damped tracking, minimal overshoot, &lt;2% settling time.</i>
    </td>
  </tr>
</table>

### Manipulator Tracking Demo
The complete physical implementation running through a multi-axis tracking profile in the Simscape Mechanics Explorer:

![Manipulator Tracking Demo](pics/arm_movement.gif)
