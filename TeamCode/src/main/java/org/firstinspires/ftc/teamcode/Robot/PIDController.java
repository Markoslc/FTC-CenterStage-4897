package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    private        double      kp;
    private        double      ki;
    private        double      kd;
    private static double      lastError;
    private static double      errorSum;
    private final  ElapsedTime timer = new ElapsedTime();

    /**
     * Constructor for PIDController
     *
     * @param kp Proportional coefficient
     * @param ki Integral coefficient
     * @param kd Derivative coefficient
     */
    public PIDController(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;

        lastError = 0;
        errorSum = 0;
    }

    /**
     * Updates the coefficients of the PID controller
     *
     * @param kp Proportional coefficient
     * @param ki Integral coefficient
     * @param kd Derivative coefficient
     */
    public void updateCoefficients(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }

    /**
     * Returns the power output of the PID controller
     *
     * @param targetVelocity The desired velocity
     * @param currVelocity   The current velocity
     * @return The power output of the PID controller
     */
    public double getPowerPID(double targetVelocity, double currVelocity) {
        double time = timer.seconds();
        timer.reset();

        double error = targetVelocity - currVelocity;

        errorSum += error * time;
        double errorChange = (error - lastError) / time;

        lastError = error;

        return error * kp + errorSum * ki + errorChange * kd;
    }
}
