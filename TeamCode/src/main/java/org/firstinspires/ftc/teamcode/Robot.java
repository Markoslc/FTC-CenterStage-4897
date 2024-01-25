package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotParameters.*;
import static org.firstinspires.ftc.teamcode.RobotParameters.Systems;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Robot {
    //
    // Robot
    //
    public static DriveMode driveMode;

    //
    // Wheels
    //
    public static DcMotorEx frontL;
    public static DcMotorEx frontR;
    public static DcMotorEx backL;
    public static DcMotorEx backR;
    public static boolean wheelsBusy = false;

    //
    // Arm
    //
    public static DcMotorEx arm;
    public static boolean armBusy = false;
    public static int armPosIndex = 0;

    //
    // Lift
    //
    public static DcMotorEx lift;
    public static boolean liftBusy;

    //
    // Claws
    //
    public static Servo leftClaw;
    public static Servo rightClaw;
    public static double leftClawTargetPos;
    public static double rightClawTargetPos;
    public static boolean clawsBusy = false;

    //
    // Systems
    //
    public static IMU imu;

    public Robot(boolean resetIMUYaw, double wheelPower, LinearOpMode opMode) {
        //
        // Robot
        //
        setDriveMode(DEFAULT_DRIVE_MODE);

        //
        // Wheels
        //
        frontL = opMode.hardwareMap.get(DcMotorEx.class, FRONT_LEFT_STR);
        frontR = opMode.hardwareMap.get(DcMotorEx.class, FRONT_RIGHT_STR);
        backL = opMode.hardwareMap.get(DcMotorEx.class, BACK_LEFT_STR);
        backR = opMode.hardwareMap.get(DcMotorEx.class, BACK_RIGHT_STR);

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

        //
        // Arm
        //
        arm = opMode.hardwareMap.get(DcMotorEx.class, ARM_STR);

        arm.setDirection(ARM_REVERSED ? DcMotorEx.Direction.REVERSE : DcMotorEx.Direction.FORWARD);

        arm.setTargetPosition(DEFAULT_ARM_POS);

        arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        arm.setPower(DEFAULT_ARM_POWER);

        //
        // Lift
        //
        lift = opMode.hardwareMap.get(DcMotorEx.class, LIFT_STR);

        lift.setDirection(LIFT_REVERSED ? DcMotorEx.Direction.REVERSE : DcMotorEx.Direction.FORWARD);

        lift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        lift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        lift.setPower(0);

        //
        // Claws
        //
        leftClaw = opMode.hardwareMap.get(Servo.class, LEFT_CLAW_STR);
        rightClaw = opMode.hardwareMap.get(Servo.class, RIGHT_CLAW_STR);

        leftClaw.setDirection(LEFT_CLAW_REVERSED ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        rightClaw.setDirection(RIGHT_CLAW_REVERSED ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);

        //
        // Systems
        //
        imu = opMode.hardwareMap.get(IMU.class, IMU_STR);
        imu.initialize(IMU_DEFAULT_PARAMS);
        if(resetIMUYaw) imu.resetYaw();
    }

    //
    // Robot
    //
    public void setDriveMode(DriveMode driveMode){
        Robot.driveMode = driveMode;
    }
    public void updateStates(){
        wheelsBusy = false;
        armBusy = false;
        liftBusy = false;
        clawsBusy = false;
    }

    //
    // Wheels
    //
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
        double frontLPower;
        double frontRPower;
        double backLPower;
        double backRPower;

        wheelsBusy = false;
        switch (driveMode){
            case ROBOT:
                frontLPower = Range.clip(forwardPower + sidePower + rotationPower, -1, 1);
                frontRPower = Range.clip(forwardPower - sidePower - rotationPower, -1, 1);
                backLPower = Range.clip(forwardPower - sidePower + rotationPower, -1, 1);
                backRPower = Range.clip(forwardPower + sidePower - rotationPower, -1, 1);

                frontL.setVelocity(WHEEL_DEGREES_PER_SECOND * frontLPower, AngleUnit.DEGREES);
                frontR.setVelocity(WHEEL_DEGREES_PER_SECOND * frontRPower, AngleUnit.DEGREES);
                backL.setVelocity(WHEEL_DEGREES_PER_SECOND * backLPower, AngleUnit.DEGREES);
                backR.setVelocity(WHEEL_DEGREES_PER_SECOND * backRPower, AngleUnit.DEGREES);

                break;
            case FIELD:
                double angle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                forwardPower *= Math.sin(angle);
                sidePower *= Math.cos(angle);

                frontLPower = Range.clip(forwardPower + sidePower + rotationPower, -1, 1);
                frontRPower = Range.clip(forwardPower - sidePower - rotationPower, -1, 1);
                backLPower = Range.clip(forwardPower - sidePower + rotationPower, -1, 1);
                backRPower = Range.clip(forwardPower + sidePower - rotationPower, -1, 1);

                frontL.setVelocity(WHEEL_DEGREES_PER_SECOND * frontLPower, AngleUnit.DEGREES);
                frontR.setVelocity(WHEEL_DEGREES_PER_SECOND * frontRPower, AngleUnit.DEGREES);
                backL.setVelocity(WHEEL_DEGREES_PER_SECOND * backLPower, AngleUnit.DEGREES);
                backR.setVelocity(WHEEL_DEGREES_PER_SECOND * backRPower, AngleUnit.DEGREES);
                break;
            default:
                frontL.setVelocity(0);
                frontR.setVelocity(0);
                backL.setVelocity(0);
                backR.setVelocity(0);
        }
        if(forwardPower != 0 || sidePower != 0 || rotationPower != 0) wheelsBusy = true;
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

    //
    // Arm
    //
    public void moveArm(int targetPos){
        arm.setTargetPosition(targetPos);

        armBusy = true;
    }
    public void nextArmPos(){
        armPosIndex++;

        armPosIndex = Math.min(armPosIndex, ArmPositions.length - 1);

        moveArm(ArmPositions[armPosIndex]);
    }
    public void prevArmPos(){
        armPosIndex--;

        armPosIndex = Math.max(armPosIndex, 0);

        moveArm(ArmPositions[armPosIndex]);
    }
    public void waitForArm(){
        while(armBusy){
            armBusy = arm.isBusy();
        }
    }

    //
    // Lift
    //
    public void moveLift(LiftDirections direction){
        switch (direction){
            case UP:
                lift.setPower(1);
                liftBusy = true;
                break;
            case DOWN:
                lift.setPower(-1);
                liftBusy = true;
                break;
        }
    }
    public void waitForLift(){
        while(liftBusy){
            liftBusy = lift.isBusy();
        }
    }

    //
    // Claws
    //
    public void moveClaws(boolean moveLeft, boolean moveRight, ClawPositions clawPos){
        leftClawTargetPos = ClawPositions.leftClaw(clawPos);
        rightClawTargetPos = ClawPositions.leftClaw(clawPos);
        
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

    //
    // Systems
    //
    public void waitForSystem(int extraTime, Systems... systems){
        for (Systems system : systems) {
            switch (system){
                case WHEELS:
                    waitForWheels();
                    break;
                case ARM:
                    waitForArm();
                    break;
                case LIFT:
                    waitForLift();
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
    public void telemetrySystems(LinearOpMode opMode, Systems... systems){
        for (Systems system : systems){
            switch (system) {
                case WHEELS:
                    opMode.telemetry.addData("Wheels busy: ", wheelsBusy);
                    break;
                case ARM:
                    opMode.telemetry.addData("Arm busy: ", armBusy);
                    break;
                case LIFT:
                    opMode.telemetry.addData("Lift busy: ", liftBusy);
                    break;
                case CLAWS:
                    opMode.telemetry.addData("Claws busy:", clawsBusy);
                    break;
                case IMU:
                    opMode.telemetry.addData("IMU rotation:", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
                case ALL:
                    telemetrySystems(opMode, Systems.WHEELS, Systems.ARM, Systems.LIFT, Systems.CLAWS, Systems.IMU);
            }
        }
        opMode.telemetry.update();
    }
}
