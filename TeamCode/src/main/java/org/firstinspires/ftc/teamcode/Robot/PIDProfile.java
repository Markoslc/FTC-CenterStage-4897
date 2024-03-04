package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDProfile {
    private final PIDController pidController;
    private final double        maxAcceleration;
    private final ElapsedTime   timer = new ElapsedTime();

    /**
     * Constructor for PIDProfile
     *
     * @param kp              Proportional coefficient
     * @param ki              Integral coefficient
     * @param kd              Derivative coefficient
     * @param maxAcceleration The maximum acceleration
     */
    public PIDProfile(double kp, double ki, double kd, double maxAcceleration) {
        pidController = new PIDController(kp, ki, kd);

        this.maxAcceleration = maxAcceleration;
    }

    /**
     * Updates the coefficients of the PID controller
     *
     * @param kp Proportional coefficient
     * @param ki Integral coefficient
     * @param kd Derivative coefficient
     */
    public void updateCoefficients(double kp, double ki, double kd) {
        pidController.updateCoefficients(kp, ki, kd);
    }

    /**
     * Returns the power output of the PID controller
     *
     * @param targetVelocity The desired velocity
     * @param currVelocity   The current velocity
     * @return The power output of the PID controller
     */
    public double getPower(double targetVelocity, double currVelocity) {
        double time = timer.seconds();
        timer.reset();

        double acceleration;
        double velocityError = targetVelocity - currVelocity;
        if (Math.abs(velocityError) < maxAcceleration * time) {
            acceleration = velocityError / time;
        } else {
            acceleration = Math.copySign(maxAcceleration, velocityError);
        }

        currVelocity += acceleration * time;

        return pidController.getPower(targetVelocity, currVelocity);
    }
}