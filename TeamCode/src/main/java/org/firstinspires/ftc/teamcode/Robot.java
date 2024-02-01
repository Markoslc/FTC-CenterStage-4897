package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotParameters.*;

import static java.lang.Thread.sleep;

import androidx.annotation.Nullable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Robot {
    //
    // Robot
    //
    public static DriveMode    driveMode;
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
    public static int       armPosIndex = 0;

    //
    // Lift
    //
    public static DcMotorEx      lift;
    public static LiftDirections currLiftDirection = LiftDirections.REST;
    public static LiftDirections nextLiftDirection = LiftDirections.UP;

    //
    // Claws
    //
    public static Servo   leftClaw;
    public static Servo   rightClaw;
    public static double  leftClawTargetPos;
    public static double  rightClawTargetPos;
    public static boolean clawsBusy;

    //
    // Systems
    //
    public static IMU    imu;
    public static double currImuTargetAngle;

    public Robot(boolean resetIMUYaw, LinearOpMode opMode) {
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

        frontL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        frontL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        frontL.setPower(0);
        frontR.setPower(0);
        backL.setPower(0);
        backR.setPower(0);


        //
        // Arm
        //
        arm = opMode.hardwareMap.get(DcMotorEx.class, ARM_STR);

        arm.setDirection(ARM_REVERSED ? DcMotorEx.Direction.REVERSE : DcMotorEx.Direction.FORWARD);

        arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        arm.setTargetPosition(0);

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
        if (resetIMUYaw) imu.resetYaw();
    }

    //
    // Robot
    //
    public void setDriveMode(DriveMode driveMode) {
        Robot.driveMode = driveMode;
    }

    /**
     * Update telemetry. Leave blank for update of all systems
     *
     * @param systems The systems that will be updated in the telemetry
     */
    public void update(@Nullable Systems... systems) {
        if (systems != null && systems.length > 0) {
            for (Systems system : systems) {
                switch (system) {
                    case WHEELS:
                        currOpMode.telemetry.addData("FL busy:", frontL.isBusy());
                        currOpMode.telemetry.addData("FR busy:", frontR.isBusy());
                        currOpMode.telemetry.addData("BL busy:", backL.isBusy());
                        currOpMode.telemetry.addData("BR busy:", backR.isBusy());

                        currOpMode.telemetry.addData("FL velocity:", frontL.getVelocity(AngleUnit.DEGREES));
                        currOpMode.telemetry.addData("FR velocity:", frontR.getVelocity(AngleUnit.DEGREES));
                        currOpMode.telemetry.addData("BL velocity:", backL.getVelocity(AngleUnit.DEGREES));
                        currOpMode.telemetry.addData("BR velocity:", backR.getVelocity(AngleUnit.DEGREES));
                        break;
                    case ARM:
                        currOpMode.telemetry.addData("Arm busy:", arm.isBusy());
                        currOpMode.telemetry.addData("Arm position:", arm.getCurrentPosition());

                        switch (armPosIndex) {
                            case 0:
                                currOpMode.telemetry.addLine("Arm target position: Rest");
                                break;
                            case 1:
                                currOpMode.telemetry.addLine("Arm target position: Score");
                                break;
                            case 2:
                                currOpMode.telemetry.addLine("Arm target position: Load");
                                break;
                        }
                        break;
                    case LIFT:
                        currOpMode.telemetry.addData("Lift busy:", lift.isBusy());
                        currOpMode.telemetry.addData("Lift power:", lift.getPower());
                        switch (currLiftDirection) {
                            case UP:
                                currOpMode.telemetry.addLine("Lift direction: Up");
                                break;
                            case DOWN:
                                currOpMode.telemetry.addLine("Lift direction: Down");
                                break;
                            case REST:
                                currOpMode.telemetry.addLine("Lift direction: Rest");
                                break;
                        }
                        break;
                    case CLAWS:
                        currOpMode.telemetry.addData("Claws busy", clawsBusy);
                        currOpMode.telemetry.addData("LeftClaw Position: ", leftClaw.getPosition());
                        currOpMode.telemetry.addData("RightClaw Position: ", rightClaw.getPosition());

                        clawsBusy = false;
                        break;
                    case IMU:
                        currOpMode.telemetry.addData("IMU angle:", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
                        break;
                }
            }
        } else update(Systems.WHEELS, Systems.ARM, Systems.LIFT, Systems.CLAWS, Systems.IMU);
        currOpMode.telemetry.update();
    }

    //
    // Wheels
    //
    public void positionDrive(int frontLRot, int frontRRot, int backLRot, int backRRot) {
        int frontLTargetPos = frontL.getCurrentPosition() + frontLRot;
        int frontRTargetPos = frontR.getCurrentPosition() + frontRRot;
        int backLTargetPos  = backL.getCurrentPosition() + backLRot;
        int backRTargetPos  = backR.getCurrentPosition() + backRRot;

        frontL.setTargetPosition(frontLTargetPos);
        frontR.setTargetPosition(frontRTargetPos);
        backL.setTargetPosition(backLTargetPos);
        backR.setTargetPosition(backRTargetPos);
    }

    /**
     * Moves the robot depending on the current drive mode
     *
     * @param forwardPower  The multiplier of the movement in the forward/backward direction. Ranges from -1 to 1
     * @param sidePower     The multiplier of the movement in the left/right direction. Ranges from -1 to 1
     * @param rotationPower The multiplier of the turning. Ranges from -1 to 1
     */
    public void drive(double forwardPower, double sidePower, double rotationPower) {

        double denominator = Math.max(Math.abs(forwardPower) + Math.abs(sidePower) + Math.abs(sidePower), 1);

        double frontLPower;
        double frontRPower;
        double backLPower;
        double backRPower;

        rotationPower *= ROTATION_POWER_MULTIPLIER;

        switch (driveMode) {
            case ROBOT:
                sidePower *= SIDE_POWER_PERFECTION_MULTIPLIER;

                frontLPower = forwardPower + sidePower + rotationPower;
                frontRPower = forwardPower - sidePower - rotationPower;
                backLPower = forwardPower - sidePower + rotationPower;
                backRPower = forwardPower + sidePower - rotationPower;

                frontL.setPower(frontLPower / denominator);
                frontR.setPower(frontRPower / denominator);
                backL.setPower(backLPower / denominator);
                backR.setPower(backRPower / denominator);

                break;
            case FIELD:
                // double stickAngle = Math.atan2(forwardPower, sidePower);
                double robotRotation = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                forwardPower = sidePower * Math.cos(-robotRotation) - forwardPower * Math.sin(-robotRotation);
                sidePower = sidePower * Math.sin(-robotRotation) + forwardPower * Math.cos(-robotRotation);
                sidePower *= SIDE_POWER_PERFECTION_MULTIPLIER;

                frontLPower = forwardPower + sidePower + rotationPower;
                frontRPower = forwardPower - sidePower - rotationPower;
                backLPower = forwardPower - sidePower + rotationPower;
                backRPower = forwardPower + sidePower - rotationPower;

                frontL.setPower(frontLPower / denominator);
                frontR.setPower(frontRPower / denominator);
                backL.setPower(backLPower / denominator);
                backR.setPower(backRPower / denominator);
                break;
            default:
                frontL.setPower(0);
                frontR.setPower(0);
                backL.setPower(0);
                backR.setPower(0);
        }
    }

    public void moveForward(int rotation, @Nullable boolean... individualWait) {
        int frontLTargetPos = frontL.getCurrentPosition() + rotation;
        int frontRTargetPos = frontR.getCurrentPosition() + rotation;
        int backLTargetPos  = backL.getCurrentPosition() + rotation;
        int backRTargetPos  = backR.getCurrentPosition() + rotation;

        frontL.setTargetPosition(frontLTargetPos);
        frontR.setTargetPosition(frontRTargetPos);
        backL.setTargetPosition(backLTargetPos);
        backR.setTargetPosition(backRTargetPos);

        if (individualWait != null && individualWait.length > 0 && individualWait[0]) {
            waitForSystem(20, Systems.WHEELS);
        }
    }

    public void moveBackward(int rotation, @Nullable boolean... individualWait) {
        int frontLTargetPos = frontL.getCurrentPosition() - rotation;
        int frontRTargetPos = frontR.getCurrentPosition() - rotation;
        int backLTargetPos  = backL.getCurrentPosition() - rotation;
        int backRTargetPos  = backR.getCurrentPosition() - rotation;

        frontL.setTargetPosition(frontLTargetPos);
        frontR.setTargetPosition(frontRTargetPos);
        backL.setTargetPosition(backLTargetPos);
        backR.setTargetPosition(backRTargetPos);

        if (individualWait != null && individualWait.length > 0 && individualWait[0]) {
            waitForSystem(20, Systems.WHEELS);
        }
    }

    public void moveLeft(int rotation, @Nullable boolean... individualWait) {
        int frontLTargetPos = frontL.getCurrentPosition() - rotation;
        int frontRTargetPos = frontR.getCurrentPosition() + rotation;
        int backLTargetPos  = backL.getCurrentPosition() + rotation;
        int backRTargetPos  = backR.getCurrentPosition() - rotation;

        frontL.setTargetPosition(frontLTargetPos);
        frontR.setTargetPosition(frontRTargetPos);
        backL.setTargetPosition(backLTargetPos);
        backR.setTargetPosition(backRTargetPos);

        if (individualWait != null && individualWait.length > 0 && individualWait[0]) {
            waitForSystem(20, Systems.WHEELS);
        }
    }

    public void moveRight(int rotation, @Nullable boolean... individualWait) {
        int frontLTargetPos = frontL.getCurrentPosition() + rotation;
        int frontRTargetPos = frontR.getCurrentPosition() - rotation;
        int backLTargetPos  = backL.getCurrentPosition() - rotation;
        int backRTargetPos  = backR.getCurrentPosition() + rotation;

        frontL.setTargetPosition(frontLTargetPos);
        frontR.setTargetPosition(frontRTargetPos);
        backL.setTargetPosition(backLTargetPos);
        backR.setTargetPosition(backRTargetPos);

        if (individualWait != null && individualWait.length > 0 && individualWait[0]) {
            waitForSystem(20, Systems.WHEELS);
        }
    }

    public void turnLeft(int rotation, @Nullable boolean... individualWait) {
        int frontLTargetPos = frontL.getCurrentPosition() - rotation;
        int frontRTargetPos = frontR.getCurrentPosition() + rotation;
        int backLTargetPos  = backL.getCurrentPosition() - rotation;
        int backRTargetPos  = backR.getCurrentPosition() + rotation;

        frontL.setTargetPosition(frontLTargetPos);
        frontR.setTargetPosition(frontRTargetPos);
        backL.setTargetPosition(backLTargetPos);
        backR.setTargetPosition(backRTargetPos);

        if (individualWait != null && individualWait.length > 0 && individualWait[0]) {
            waitForSystem(20, Systems.WHEELS);
        }
    }

    public void turnRight(int rotation, @Nullable boolean... individualWait) {
        int frontLTargetPos = frontL.getCurrentPosition() + rotation;
        int frontRTargetPos = frontR.getCurrentPosition() - rotation;
        int backLTargetPos  = backL.getCurrentPosition() + rotation;
        int backRTargetPos  = backR.getCurrentPosition() - rotation;

        frontL.setTargetPosition(frontLTargetPos);
        frontR.setTargetPosition(frontRTargetPos);
        backL.setTargetPosition(backLTargetPos);
        backR.setTargetPosition(backRTargetPos);

        if (individualWait != null && individualWait.length > 0 && individualWait[0]) {
            waitForSystem(20, Systems.WHEELS);
        }
    }

    public void turnAngleLeft(double angle, @Nullable boolean... individualWait) {
        double imuAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        if (imuAngle + angle > 180) {
            angle %= 180;
            currImuTargetAngle = -180 + ((imuAngle + angle) % 180);
        }

        frontL.setVelocity(-WHEEL_DEGREES_PER_SECOND, AngleUnit.DEGREES);
        frontR.setVelocity(WHEEL_DEGREES_PER_SECOND, AngleUnit.DEGREES);
        backL.setVelocity(-WHEEL_DEGREES_PER_SECOND, AngleUnit.DEGREES);
        backR.setVelocity(WHEEL_DEGREES_PER_SECOND, AngleUnit.DEGREES);

        if (individualWait != null && individualWait.length > 0 && individualWait[0]) {
            waitForSystem(20, Systems.IMU);
        }
    }

    public void turnAngleRight(double angle, @Nullable boolean... individualWait) {
        double imuAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        if (imuAngle - angle < -180) {
            currImuTargetAngle = 180 - Math.abs((imuAngle - angle) % 180);
        }

        frontL.setVelocity(WHEEL_DEGREES_PER_SECOND, AngleUnit.DEGREES);
        frontR.setVelocity(-WHEEL_DEGREES_PER_SECOND, AngleUnit.DEGREES);
        backL.setVelocity(WHEEL_DEGREES_PER_SECOND, AngleUnit.DEGREES);
        backR.setVelocity(-WHEEL_DEGREES_PER_SECOND, AngleUnit.DEGREES);

        if (individualWait != null && individualWait.length > 0 && individualWait[0]) {
            waitForSystem(20, Systems.IMU);
        }
    }

    public void waitForWheels() {
        while (frontL.isBusy() || frontR.isBusy() || backL.isBusy() || backR.isBusy()) {
            currOpMode.sleep(1);
        }
    }

    /**
     * Resets the wheels' encoders
     *
     * @param nextMode Sets the next mode for the wheels. Leave blank for same mode as before the reset
     */
    public void resetWheelEncoders(@Nullable DcMotorEx.RunMode... nextMode) {
        if (nextMode != null && nextMode.length == 0)
            nextMode = new DcMotorEx.RunMode[]{frontL.getMode()};

        frontL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        frontL.setMode(nextMode != null ? nextMode[0] : DcMotorEx.RunMode.RUN_TO_POSITION);
        frontR.setMode(nextMode != null ? nextMode[0] : DcMotorEx.RunMode.RUN_TO_POSITION);
        backL.setMode(nextMode != null ? nextMode[0] : DcMotorEx.RunMode.RUN_TO_POSITION);
        backR.setMode(nextMode != null ? nextMode[0] : DcMotorEx.RunMode.RUN_TO_POSITION);
    }

    //
    // Arm
    //
    public void moveArm(int targetPos, @Nullable boolean... individualWait) {
        arm.setTargetPosition(targetPos);

        if (individualWait != null && individualWait.length > 0 && individualWait[0]) {
            waitForSystem(20, Systems.ARM);
        }
    }

    public void nextArmPos() {
        armPosIndex++;

        armPosIndex = Math.min(armPosIndex, ArmPositions.length - 1);

        moveArm(ArmPositions[armPosIndex]);
    }

    public void prevArmPos() {
        armPosIndex--;

        armPosIndex = Math.max(armPosIndex, 0);

        moveArm(ArmPositions[armPosIndex]);
    }

    public void waitForArm() {
        while (arm.isBusy()) {
            currOpMode.sleep(1);
        }
    }

    //
    // Lift
    //
    public void moveLift(LiftDirections direction) {
        switch (direction) {
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

    public void waitForLift() {
        while (lift.isBusy()) {
            currOpMode.sleep(1);
        }
    }

    //
    // Claws
    //
    public void moveClaws(boolean moveLeft, boolean moveRight, ClawPositions clawPos, @Nullable boolean... individualWait) {
        if (moveLeft) leftClawTargetPos = ClawPositions.leftClaw(clawPos);
        if (moveRight) rightClawTargetPos = ClawPositions.rightClaw(clawPos);

        leftClaw.setPosition(leftClawTargetPos);
        rightClaw.setPosition(rightClawTargetPos);

        clawsBusy = true;

        if (individualWait != null && individualWait.length > 0 && individualWait[0]) {
            waitForSystem(20, Systems.CLAWS);
        }
    }

    public void waitForClaws() {
        while (clawsBusy) {
            boolean leftClawBusy  = leftClaw.getPosition() >= leftClawTargetPos + 1 || leftClaw.getPosition() <= leftClawTargetPos - 1;
            boolean rightClawBusy = rightClaw.getPosition() >= rightClawTargetPos + 1 || rightClaw.getPosition() <= rightClawTargetPos - 1;

            clawsBusy = leftClawBusy || rightClawBusy;
        }
    }

    //
    // Systems
    //
    private void waitForImu() {
        double angle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        while (angle > currImuTargetAngle + IMU_TOLERANCE_DEGREES || angle < currImuTargetAngle - IMU_TOLERANCE_DEGREES) {
            currOpMode.sleep(1);

            angle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        }

        frontL.setPower(0);
        frontR.setPower(0);
        backL.setPower(0);
        backR.setPower(0);
    }

    public void waitForSystem(int extraTime, Systems... systems) {
        for (Systems system : systems) {
            switch (system) {
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
