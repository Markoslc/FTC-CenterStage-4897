package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotParameters.*;

import static java.lang.Thread.sleep;

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
    public static LinearOpMode currOpMode;

    //
    // Wheels
    //
    public static DcMotorEx frontL;
    public static DcMotorEx frontR;
    public static DcMotorEx backL;
    public static DcMotorEx backR;

    //
    // Arm
    //
    public static DcMotorEx arm;
    public static int armPosIndex = 0;

    //
    // Lift
    //
    public static DcMotorEx lift;
    public static LiftDirections currLiftDirection = LiftDirections.REST;
    public static LiftDirections nextLiftDirection = LiftDirections.UP;

    //
    // Claws
    //
    public static Servo leftClaw;
    public static Servo rightClaw;
    public static double leftClawTargetPos;
    public static double rightClawTargetPos;
    public static boolean clawsBusy;

    //
    // Systems
    //
    public static IMU imu;
    public static double currImuTargetAngle;

    public Robot(boolean resetIMUYaw, double wheelPower, LinearOpMode opMode) {
        //
        // Robot
        //
        setDriveMode(DEFAULT_DRIVE_MODE);
        currOpMode = opMode;

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

        arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        arm.setTargetPosition(0);

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
    /**
    * Update telemetry. Leave blank for update of all systems
    * @param systems The systems that will be updated in the telemetry
     */
    public void update(Systems... systems){
        for(Systems system : systems){
            switch (system){
                case WHEELS:
                    currOpMode.telemetry.addData("FL busy:", frontL.isBusy());
                    currOpMode.telemetry.addData("FR busy:", frontR.isBusy());
                    currOpMode.telemetry.addData("BL busy:", backL.isBusy());
                    currOpMode.telemetry.addData("BR busy:", backR.isBusy());
                    break;
                case ARM:
                    currOpMode.telemetry.addData("Arm busy:", arm.isBusy());
                    break;
                case LIFT:
                    currOpMode.telemetry.addData("Lift busy:", lift.isBusy());
                    switch (currLiftDirection){
                        case UP:
                            currOpMode.telemetry.addLine("Lift direction: Up");
                            break;
                        case DOWN:
                            currOpMode.telemetry.addLine("Lift direction: Down");
                            break;
                        default:
                            currOpMode.telemetry.addLine("Lift direction: Rest");
                    }
                    break;
                case CLAWS:
                    currOpMode.telemetry.addData("Claws busy", clawsBusy);
                    clawsBusy = false;
                    break;
                case IMU:
                    currOpMode.telemetry.addData("IMU angle:", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
                    break;
                default:
                    update(Systems.WHEELS, Systems.ARM, Systems.LIFT, Systems.CLAWS, Systems.IMU);
            }
        }
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
    }
    /**
     * Moves the robot depending on the current drive mode
     * @param forwardPower The multiplier of the movement in the forward/backward direction. Ranges from -1 to 1
     * @param sidePower The multiplier of the movement in the left/right direction. Ranges from -1 to 1
     * @param rotationPower The multiplier of the turning. Ranges from -1 to 1
     */
    public void drive(double forwardPower, double sidePower, double rotationPower){
        double frontLPower;
        double frontRPower;
        double backLPower;
        double backRPower;

        rotationPower *= ROTATION_POWER_MULTIPLIER;

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
                double stickAngle = Math.atan2(forwardPower, sidePower);
                double angle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - stickAngle;

                forwardPower *= Math.cos(Math.toRadians(angle));
                sidePower *= Math.sin(Math.toRadians(angle));

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
    }
    public void moveForward(int rotation, boolean... individualWait){
        int frontLTargetPos = frontL.getCurrentPosition() + rotation;
        int frontRTargetPos = frontR.getCurrentPosition() + rotation;
        int backLTargetPos = backL.getCurrentPosition() + rotation;
        int backRTargetPos = backR.getCurrentPosition() + rotation;

        frontL.setTargetPosition(frontLTargetPos);
        frontR.setTargetPosition(frontRTargetPos);
        backL.setTargetPosition(backLTargetPos);
        backR.setTargetPosition(backRTargetPos);

        if(individualWait[0]) waitForSystem(20, Systems.WHEELS);
    }
    public void moveBackward(int rotation, boolean... individualWait){
        int frontLTargetPos = frontL.getCurrentPosition() - rotation;
        int frontRTargetPos = frontR.getCurrentPosition() - rotation;
        int backLTargetPos = backL.getCurrentPosition() - rotation;
        int backRTargetPos = backR.getCurrentPosition() - rotation;

        frontL.setTargetPosition(frontLTargetPos);
        frontR.setTargetPosition(frontRTargetPos);
        backL.setTargetPosition(backLTargetPos);
        backR.setTargetPosition(backRTargetPos);

        if(individualWait[0]) waitForSystem(20, Systems.WHEELS);
    }
    public void moveLeft(int rotation, boolean... individualWait){
        int frontLTargetPos = frontL.getCurrentPosition() - rotation;
        int frontRTargetPos = frontR.getCurrentPosition() + rotation;
        int backLTargetPos = backL.getCurrentPosition() + rotation;
        int backRTargetPos = backR.getCurrentPosition() - rotation;

        frontL.setTargetPosition(frontLTargetPos);
        frontR.setTargetPosition(frontRTargetPos);
        backL.setTargetPosition(backLTargetPos);
        backR.setTargetPosition(backRTargetPos);

        if(individualWait[0]) waitForSystem(20, Systems.WHEELS);
    }
    public void moveRight(int rotation, boolean... individualWait){
        int frontLTargetPos = frontL.getCurrentPosition() + rotation;
        int frontRTargetPos = frontR.getCurrentPosition() - rotation;
        int backLTargetPos = backL.getCurrentPosition() - rotation;
        int backRTargetPos = backR.getCurrentPosition() + rotation;

        frontL.setTargetPosition(frontLTargetPos);
        frontR.setTargetPosition(frontRTargetPos);
        backL.setTargetPosition(backLTargetPos);
        backR.setTargetPosition(backRTargetPos);

        if(individualWait[0]) waitForSystem(20, Systems.WHEELS);
    }
    public void turnLeft(int rotation, boolean... individualWait){
        int frontLTargetPos = frontL.getCurrentPosition() - rotation;
        int frontRTargetPos = frontR.getCurrentPosition() + rotation;
        int backLTargetPos = backL.getCurrentPosition() - rotation;
        int backRTargetPos = backR.getCurrentPosition() + rotation;

        frontL.setTargetPosition(frontLTargetPos);
        frontR.setTargetPosition(frontRTargetPos);
        backL.setTargetPosition(backLTargetPos);
        backR.setTargetPosition(backRTargetPos);

        if(individualWait[0]) waitForSystem(20, Systems.WHEELS);
    }
    public void turnRight(int rotation, boolean... individualWait){
        int frontLTargetPos = frontL.getCurrentPosition() + rotation;
        int frontRTargetPos = frontR.getCurrentPosition() - rotation;
        int backLTargetPos = backL.getCurrentPosition() + rotation;
        int backRTargetPos = backR.getCurrentPosition() - rotation;

        frontL.setTargetPosition(frontLTargetPos);
        frontR.setTargetPosition(frontRTargetPos);
        backL.setTargetPosition(backLTargetPos);
        backR.setTargetPosition(backRTargetPos);

        if(individualWait[0]) waitForSystem(20, Systems.WHEELS);
    }
    public void turnAngleLeft(double angle, boolean... individualWait){
        double imuAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        if(imuAngle + angle > 180){
            angle %= 180;
            currImuTargetAngle = -180 + ((imuAngle + angle) % 180);
        }

        frontL.setVelocity(-WHEEL_DEGREES_PER_SECOND, AngleUnit.DEGREES);
        frontR.setVelocity(WHEEL_DEGREES_PER_SECOND, AngleUnit.DEGREES);
        backL.setVelocity(-WHEEL_DEGREES_PER_SECOND, AngleUnit.DEGREES);
        backR.setVelocity(WHEEL_DEGREES_PER_SECOND, AngleUnit.DEGREES);

        if(individualWait[0]) waitForSystem(20, Systems.IMU);
    }
    public void turnAngleRight(double angle, boolean... individualWait){
        double imuAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        if(imuAngle - angle < -180){
            currImuTargetAngle = 180 - Math.abs((imuAngle - angle) % 180);
        }

        frontL.setVelocity(WHEEL_DEGREES_PER_SECOND, AngleUnit.DEGREES);
        frontR.setVelocity(-WHEEL_DEGREES_PER_SECOND, AngleUnit.DEGREES);
        backL.setVelocity(WHEEL_DEGREES_PER_SECOND, AngleUnit.DEGREES);
        backR.setVelocity(-WHEEL_DEGREES_PER_SECOND, AngleUnit.DEGREES);

        if(individualWait[0]) waitForSystem(20, Systems.IMU);
    }
    public void waitForWheels(){
        while(frontL.isBusy() || frontR.isBusy() || backL.isBusy() || backR.isBusy()){
            currOpMode.sleep(1);
        }
    }
    public void resetWheelEncoders(DcMotorEx.RunMode... nextMode){
        if(nextMode[0] == null) nextMode[0] = frontL.getMode();

        frontL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        frontL.setMode(nextMode[0]);
        frontR.setMode(nextMode[0]);
        backL.setMode(nextMode[0]);
        backR.setMode(nextMode[0]);
    }

    //
    // Arm
    //
    public void moveArm(int targetPos, boolean... individualWait){
        arm.setTargetPosition(targetPos);

        if(individualWait[0]) waitForSystem(20, Systems.ARM);
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
        while(arm.isBusy()){
            currOpMode.sleep(1);
        }
    }

    //
    // Lift
    //
    public void moveLift(LiftDirections direction){
        switch (direction){
            case UP:
                lift.setPower(1);
                break;
            case DOWN:
                lift.setPower(-1);
                break;
            case REST:
                lift.setPower(0);
        }
    }
    public void waitForLift(){
        while(lift.isBusy()){
            currOpMode.sleep(1);
        }
    }

    //
    // Claws
    //
    public void moveClaws(boolean moveLeft, boolean moveRight, ClawPositions clawPos, boolean... individualWait){
        leftClawTargetPos = ClawPositions.leftClaw(clawPos);
        rightClawTargetPos = ClawPositions.leftClaw(clawPos);
        
        leftClaw.setPosition(moveLeft ? leftClawTargetPos : leftClaw.getPosition());
        rightClaw.setPosition(moveRight ? rightClawTargetPos : rightClaw.getPosition());

        clawsBusy = true;

        if(individualWait[0]) waitForSystem(20, Systems.CLAWS);
    }
    public void waitForClaws(){
        while(clawsBusy){
            boolean leftClawBusy = leftClaw.getPosition() >= leftClawTargetPos + 1 || leftClaw.getPosition() <= leftClawTargetPos - 1;
            boolean rightClawBusy = rightClaw.getPosition() >= rightClawTargetPos + 1 || rightClaw.getPosition() <= rightClawTargetPos - 1;

            clawsBusy = leftClawBusy || rightClawBusy;
        }
    }

    //
    // Systems
    //
    private void waitForImu(){
        while(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) != currImuTargetAngle){
            currOpMode.sleep(1);
        }

        frontL.setVelocity(0, AngleUnit.DEGREES);
        frontR.setVelocity(0, AngleUnit.DEGREES);
        backL.setVelocity(0, AngleUnit.DEGREES);
        backR.setVelocity(0, AngleUnit.DEGREES);
    }
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
                    break;
                case IMU:
                    waitForImu();
                    break;
                default:
                    waitForSystem(20, Systems.WHEELS, Systems.ARM, Systems.LIFT, Systems.CLAWS);
            }
        }

        try {
            sleep(extraTime);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
}
