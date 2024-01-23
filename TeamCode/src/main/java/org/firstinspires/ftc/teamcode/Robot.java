package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.teamcode.RobotParameters.*;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class Robot {
    // Robot
    public static DriveMode robotDriveMode;

    // Wheels
    public static DcMotorEx frontL;
    public static DcMotorEx frontR;
    public static DcMotorEx backL;
    public static DcMotorEx backR;
    public static boolean wheelsBusy = false;

    // Arm
    public static DcMotorEx arm;
    public static boolean armBusy = false;

    // Claws
    public static Servo leftClaw;
    public static Servo rightClaw;
    public static double leftClawTargetPos;
    public static double rightClawTargetPos;
    public static boolean clawsBusy = false;

    // Systems
    public static IMU imu;

    public Robot(boolean resetIMUYaw, double wheelPower) {
        // Robot
        setDriveMode(DEFAULT_DRIVE_MODE);

        // Wheels
        frontL = hardwareMap.get(DcMotorEx.class, FRONT_LEFT_STR);
        frontR = hardwareMap.get(DcMotorEx.class, FRONT_RIGHT_STR);
        backL = hardwareMap.get(DcMotorEx.class, BACK_LEFT_STR);
        backR = hardwareMap.get(DcMotorEx.class, BACK_RIGHT_STR);

        frontL.setDirection(FRONT_LEFT_REVERSED ? DcMotorEx.Direction.REVERSE : DcMotorEx.Direction.FORWARD);
        frontR.setDirection(FRONT_RIGHT_REVERSED ? DcMotorEx.Direction.REVERSE : DcMotorEx.Direction.FORWARD);
        backR.setDirection(BACK_LEFT_REVERSED ? DcMotorEx.Direction.REVERSE : DcMotorEx.Direction.FORWARD);
        backL.setDirection(BACK_RIGHT_REVERSED ? DcMotorEx.Direction.REVERSE : DcMotorEx.Direction.FORWARD);

        frontL.setTargetPosition(0);
        frontR.setTargetPosition(0);
        backL.setTargetPosition(0);
        backR.setTargetPosition(0);

        frontL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        frontL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        frontR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        frontL.setPower(wheelPower);
        frontR.setPower(wheelPower);
        backL.setPower(wheelPower);
        backR.setPower(wheelPower);

        // Arm
        arm = hardwareMap.get(DcMotorEx.class, ARM_STR);

        arm.setDirection(ARM_REVERSED ? DcMotorEx.Direction.REVERSE : DcMotorEx.Direction.FORWARD);

        arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        arm.setTargetPosition(0);

        arm.setPower(0.4);

        // Claws
        leftClaw = hardwareMap.get(Servo.class, LEFT_CLAW_STR);
        rightClaw = hardwareMap.get(Servo.class, RIGHT_CLAW_STR);

        leftClaw.setDirection(LEFT_CLAW_REVERSED ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        rightClaw.setDirection(RIGHT_CLAW_REVERSED ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);

        // Systems
        imu = hardwareMap.get(IMU.class, IMU_STR);
        imu.initialize(IMU_DEFAULT_PARAMS);
        if(resetIMUYaw) imu.resetYaw();
    }

    // Robot
    public void setDriveMode(DriveMode driveMode){
        robotDriveMode = driveMode;
    }

    //Wheels
    public void positionDrive(int frontLRot, int frontRRot, int backLRot, int backRRot){
        int frontLTargetPos = frontL.getCurrentPosition() + frontLRot;
        int frontRTargetPos = frontR.getCurrentPosition() + frontRRot;
        int backLTargetPos = backL.getCurrentPosition() + backLRot;
        int backRTargetPos = backR.getCurrentPosition() + backRRot;

        frontL.setTargetPosition(frontLTargetPos);
        frontR.setTargetPosition(frontRTargetPos);
        backL.setTargetPosition(backLTargetPos);
        backR.setTargetPosition(backRTargetPos);

        wheelsBusy = true;
    }
    public void drive(double forwardPower, double sidePower, double rotationPower){
        switch (robotDriveMode){
            case ROBOT:

                double frontLPower = Range.clip(forwardPower + sidePower + rotationPower, -1, 1);
                double frontRPower = Range.clip(forwardPower - sidePower - rotationPower, -1, 1);
                double backLPower = Range.clip(forwardPower - sidePower + rotationPower, -1, 1);
                double backRPower = Range.clip(forwardPower + sidePower - rotationPower, -1, 1);

                frontL.setVelocity(WHEEL_PPS * frontLPower);
                frontR.setVelocity(WHEEL_PPS * frontRPower);
                backL.setVelocity(WHEEL_PPS * backLPower);
                backR.setVelocity(WHEEL_PPS * backRPower);

                break;
            case FIELD:
                break;
        }
    }
    public void moveForward(int rotation){
        int frontLTargetPos = frontL.getCurrentPosition() + rotation;
        int frontRTargetPos = frontR.getCurrentPosition() + rotation;
        int backLTargetPos = backL.getCurrentPosition() + rotation;
        int backRTargetPos = backR.getCurrentPosition() + rotation;

        frontL.setTargetPosition(frontLTargetPos);
        frontR.setTargetPosition(frontRTargetPos);
        backL.setTargetPosition(backLTargetPos);
        backR.setTargetPosition(backRTargetPos);

        wheelsBusy = true;
    }
    public void moveBackward(int rotation){
        int frontLTargetPos = frontL.getCurrentPosition() - rotation;
        int frontRTargetPos = frontR.getCurrentPosition() - rotation;
        int backLTargetPos = backL.getCurrentPosition() - rotation;
        int backRTargetPos = backR.getCurrentPosition() - rotation;

        frontL.setTargetPosition(frontLTargetPos);
        frontR.setTargetPosition(frontRTargetPos);
        backL.setTargetPosition(backLTargetPos);
        backR.setTargetPosition(backRTargetPos);

        wheelsBusy = true;
    }
    public void moveLeft(int rotation){
        int frontLTargetPos = frontL.getCurrentPosition() - rotation;
        int frontRTargetPos = frontR.getCurrentPosition() + rotation;
        int backLTargetPos = backL.getCurrentPosition() + rotation;
        int backRTargetPos = backR.getCurrentPosition() - rotation;

        frontL.setTargetPosition(frontLTargetPos);
        frontR.setTargetPosition(frontRTargetPos);
        backL.setTargetPosition(backLTargetPos);
        backR.setTargetPosition(backRTargetPos);

        wheelsBusy = true;
    }
    public void moveRight(int rotation){
        int frontLTargetPos = frontL.getCurrentPosition() + rotation;
        int frontRTargetPos = frontR.getCurrentPosition() - rotation;
        int backLTargetPos = backL.getCurrentPosition() - rotation;
        int backRTargetPos = backR.getCurrentPosition() + rotation;

        frontL.setTargetPosition(frontLTargetPos);
        frontR.setTargetPosition(frontRTargetPos);
        backL.setTargetPosition(backLTargetPos);
        backR.setTargetPosition(backRTargetPos);

        wheelsBusy = true;
    }
    public void turnLeft(int rotation){
        int frontLTargetPos = frontL.getCurrentPosition() - rotation;
        int frontRTargetPos = frontR.getCurrentPosition() + rotation;
        int backLTargetPos = backL.getCurrentPosition() - rotation;
        int backRTargetPos = backR.getCurrentPosition() + rotation;

        frontL.setTargetPosition(frontLTargetPos);
        frontR.setTargetPosition(frontRTargetPos);
        backL.setTargetPosition(backLTargetPos);
        backR.setTargetPosition(backRTargetPos);

        wheelsBusy = true;
    }
    public void turnRight(int rotation){
        int frontLTargetPos = frontL.getCurrentPosition() + rotation;
        int frontRTargetPos = frontR.getCurrentPosition() - rotation;
        int backLTargetPos = backL.getCurrentPosition() + rotation;
        int backRTargetPos = backR.getCurrentPosition() - rotation;

        frontL.setTargetPosition(frontLTargetPos);
        frontR.setTargetPosition(frontRTargetPos);
        backL.setTargetPosition(backLTargetPos);
        backR.setTargetPosition(backRTargetPos);

        wheelsBusy = true;
    }
    public void waitForWheels(){
        while(wheelsBusy){
            boolean frontLBusy = frontL.isBusy();
            boolean frontRBusy = frontR.isBusy();
            boolean backLBusy = backL.isBusy();
            boolean backRBusy = backR.isBusy();

            wheelsBusy = frontLBusy && frontRBusy && backLBusy && backRBusy;
        }
    }
    public void resetWheelEncoders(DcMotorEx.RunMode nextMode){
        frontL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        frontL.setMode(nextMode);
        frontR.setMode(nextMode);
        backL.setMode(nextMode);
        backR.setMode(nextMode);
    }

    // Arm
    public void moveArm(int targetPos){
        arm.setTargetPosition(targetPos);

        armBusy = true;
    }
    public void waitForArm(){
        while(armBusy){
            armBusy = arm.isBusy();
        }
    }

    // Claws
    public void moveClaws(boolean moveLeft, boolean moveRight, int positionIndex){
        leftClawTargetPos = LEFT_CLAW_POS[positionIndex];
        rightClawTargetPos = RIGHT_CLAW_POS[positionIndex];
        
        leftClaw.setPosition(moveLeft ? leftClawTargetPos : leftClaw.getPosition());
        rightClaw.setPosition(moveRight ? rightClawTargetPos : rightClaw.getPosition());

        clawsBusy = true;
    }
    public void waitForClaws(){
        while(clawsBusy){
            boolean leftClawBusy = leftClaw.getPosition() != leftClawTargetPos;
            boolean rightClawBusy = rightClaw.getPosition() != rightClawTargetPos;
            clawsBusy = leftClawBusy && rightClawBusy;
        }
    }

    // Systems
    public void waitForSystem(int extraTime, systems... systems){
        for (systems system : systems) {
            switch (system){
                case WHEELS:
                    waitForWheels();
                    break;
                case ARM:
                    waitForArm();
                    break;
                case CLAWS:
                    waitForClaws();
            }
        }

        try {
            sleep(extraTime);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
}
