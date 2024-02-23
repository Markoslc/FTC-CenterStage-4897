package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDProfile {
    private final PIDController pidController;
    private final double maxAcceleration;
    private final  ElapsedTime timer = new ElapsedTime();

    public PIDProfile(double kp, double ki, double kd, double maxAcceleration) {
        pidController = new PIDController(kp, ki, kd);

        this.maxAcceleration = maxAcceleration;
    }

    public void updateCoefficients(double kp, double ki, double kd){
        pidController.updateCoefficients(kp, ki, kd);
    }

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

        return pidController.getPowerPID(targetVelocity, currVelocity);
    }
}
