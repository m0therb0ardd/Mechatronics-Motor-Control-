
Final Project for Mechatronics



I TEST 28.4.10 question 5
CONTROL GAINS: 
Kp_mA = 0.03 %/mA
Ki_mA = 0.05 %/mA s

![Itest_28.4.10.5](images/Itest_28.4.10.5.png)



CUBIC 28.4.12 QUESTION 5 
VALUES: 
Kp_pos = 8.0 dg/mA 
Ki_pos = 0.01 mA/deg s
Kd_pos = 1750.0 mA s/deg

![](images/cubic.png)


Extension: Feed Forward Control
Feedforward control is a control strategy used in systems where the controller anticipates the effect of a known disturbance or input and compensates for it before it affects the system's output. Unlike feedback control, which reacts to errors after they occur, feedforward control proactively adjusts the control input based on a model of the system and the expected disturbance.

Open-Loop Nature:
- Feedforward control is an open-loop control strategy because it does not rely on feedback (i.e., it does not measure the actual output to adjust the control input).
- Instead, it uses a model of the system to predict the required control action.

τ=Jα

where:
    τ is the required torque,
    J is the motor's moment of inertia,
    α is the desired acceleration.

then the required current (Iff​) is then:
Iff=τ/kt
where kt is the motor's torque constant.

    I Am trying to implement feed forward control to imporve trajectory tracking by calculating thre required current based on the desured acceleration from the trajectory



first i add a feed forward calculation:
//feedforward
float compute_feedforward_current(float desired_acceleration) {
    // Estimate feedforward current using a simple model
    float motor_inertia = 0.1;  // used motor valyes from trial and error --> need to figure these out 
    float torque_constant = 0.01;  // used motor valyes from trial and error --> need to figure these out

    // Torque required: τ = I * α (Newton's Second Law for rotation)
    float required_torque = motor_inertia * desired_acceleration;

    // Convert torque to current: I = τ / k_t
    float required_current = required_torque / torque_constant;

    return required_current;
}

then i modify the position control ISR And i add the feedforward current to the PID control signal

