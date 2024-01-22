package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.teamcode.RobotParameters.*;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Robot {
    // Wheels
    public static DcMotor frontL;
    public static DcMotor frontR;
    public static DcMotor backL;
    public static DcMotor backR;
    public boolean wheelsBusy = false;

    // Arm
    public static DcMotor arm;
    public boolean armBusy = false;

    // Claws
    public static Servo leftClaw;
    public static Servo rightClaw;
    public double leftClawTargetPos;
    public double rightClawTargetPos;
    public boolean clawsBusy = false;

    public Robot() {
        // Wheels
        frontL = hardwareMap.get(DcMotor.class, FRONT_LEFT_STR);
        frontR = hardwareMap.get(DcMotor.class, FRONT_RIGHT_STR);
        backL = hardwareMap.get(DcMotor.class, BACK_LEFT_STR);
        backR = hardwareMap.get(DcMotor.class, BACK_RIGHT_STR);

        frontL.setDirection(FRONT_LEFT_REVERSED ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        frontR.setDirection(FRONT_RIGHT_REVERSED ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        backR.setDirection(BACK_LEFT_REVERSED ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        backL.setDirection(BACK_RIGHT_REVERSED ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);

        frontL.setTargetPosition(0);
        frontR.setTargetPosition(0);
        backL.setTargetPosition(0);
        backR.setTargetPosition(0);

        frontL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontL.setPower(0.4);
        frontR.setPower(0.4);
        backL.setPower(0.4);
        backR.setPower(0.4);

        // Arm
        arm = hardwareMap.get(DcMotor.class, ARM_STR);

        arm.setDirection(ARM_REVERSED ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        arm.setTargetPosition(0);

        arm.setPower(0.4);

        // Claws
        leftClaw = hardwareMap.get(Servo.class, LEFT_CLAW_STR);
        rightClaw = hardwareMap.get(Servo.class, RIGHT_CLAW_STR);

        leftClaw.setDirection(LEFT_CLAW_REVERSED ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        rightClaw.setDirection(RIGHT_CLAW_REVERSED ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);

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
    public void waitForWheels(){
        while(wheelsBusy){
            boolean frontLBusy = frontL.isBusy();
            boolean frontRBusy = frontR.isBusy();
            boolean backLBusy = backL.isBusy();
            boolean backRBusy = backR.isBusy();

            wheelsBusy = frontLBusy && frontRBusy && backLBusy && backRBusy;
        }
    }
    public void resetWheelEncoders(DcMotor.RunMode nextMode){
        frontL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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
    public void waitForSystem(String... systems){
        for (String system : systems) {
            switch (system){
                case "Wheels":
                    waitForWheels();
                    break;
                case "Arm":
                    waitForArm();
                    break;
                case "Claws":
                    waitForClaws();
            }
        }
    }
}
