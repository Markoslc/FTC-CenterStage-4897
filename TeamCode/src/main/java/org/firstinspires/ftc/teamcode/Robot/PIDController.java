package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    private  double      kp;
    private  double      ki;
    private  double      kd;
    private static double      lastError;
    private static double      errorSum;
    private final  ElapsedTime timer = new ElapsedTime();

    public PIDController(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;

        lastError = 0;
        errorSum = 0;
    }

    public void setCoefficients(double kp, double ki, double kd){
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }

    public double getVelocity(double targetVelocity, double currVelocity) {
        double error = targetVelocity - currVelocity;

        errorSum += error * timer.seconds();
        double errorChange = (error - lastError) / timer.seconds();

        lastError = error;
        timer.reset();

        return error * kp + errorSum * ki + errorChange * kd;
    }
}
