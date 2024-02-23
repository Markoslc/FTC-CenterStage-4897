package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDProfile {
    private  double      kp;
    private  double      ki;
    private  double      kd;
    private final PIDController pidController;
    private final double maxAcceleration;
    private final  ElapsedTime timer = new ElapsedTime();

    public PIDProfile(double kp, double ki, double kd, double maxAcceleration) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;

        pidController = new PIDController(kp, ki, kd);

        this.maxAcceleration = maxAcceleration;
    }

    public void updateCoefficients(double kp, double ki, double kd){
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;

        pidController.updateCoefficients(kp, ki, kd);
    }

    //public double getPowerProfile(double targetVelocity, double currVelocity) {

    //}
}
