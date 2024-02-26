package org.firstinspires.ftc.teamcode.Robot;

import static org.firstinspires.ftc.teamcode.Robot.RobotParameters.*;

import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class Robot {
    //
    // Robot
    //
    private DriveMode driveMode;
    private final DrivePeriod drivePeriod;
    private final OpMode opMode;

    //
    // Wheels
    //
    public final DcMotorEx frontL;
    public final DcMotorEx frontR;
    public final DcMotorEx backL;
    public final DcMotorEx backR;
    private final PIDProfile frontLController;
    private final PIDProfile frontRController;
    private final PIDProfile backLController;
    private final PIDProfile backRController;
    private static double WHEEL_KP;
    private static double WHEEL_KI;
    private static double WHEEL_KD;
    private double wheelPower;

    //
    // Arm
    //
    private final DcMotorEx arm;
    private int armPosIndex = 0;

    //
    // Plane
    //
    private final Servo plane;
    //
    // Lift
    //
    private final DcMotorEx leftLift;
    private final DcMotorEx rightLift;
    private LiftPositions currLiftPosition = LiftPositions.UP;

    //
    // Claws
    //
    private final Servo leftClaw;
    private final Servo rightClaw;
    private double leftClawTargetPos;
    private double rightClawTargetPos;

    //
    // Systems
    //
    private final IMU imu;
    private double currImuTargetAngle;

    public Robot(DrivePeriod drivePeriod, boolean resetIMUYaw, double wheelPower, LinearOpMode opMode) {
        //
        // Robot
        //
        setDriveMode(DEFAULT_DRIVE_MODE);
        this.drivePeriod = drivePeriod;
        this.opMode = opMode;

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

        frontL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        switch (drivePeriod) {
            case DRIVER:
            case TEST:
                frontL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                frontR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                backL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                backR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

                frontL.setPower(0);
                frontR.setPower(0);
                backL.setPower(0);
                backR.setPower(0);
                break;
            case AUTONOMOUS:
                frontL.setTargetPosition(0);
                frontR.setTargetPosition(0);
                backL.setTargetPosition(0);
                backR.setTargetPosition(0);

                frontL.setTargetPositionTolerance(WHEELS_POSITION_TOLERANCE);
                frontR.setTargetPositionTolerance(WHEELS_POSITION_TOLERANCE);
                backL.setTargetPositionTolerance(WHEELS_POSITION_TOLERANCE);
                backR.setTargetPositionTolerance(WHEELS_POSITION_TOLERANCE);

                frontL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                frontR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                backL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                backR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

                frontL.setPower(wheelPower);
                frontR.setPower(wheelPower);
                backL.setPower(wheelPower);
                backR.setPower(wheelPower);
        }

        WHEEL_KP = DEFAULT_WHEEL_KP;
        WHEEL_KI = DEFAULT_WHEEL_KI;
        WHEEL_KD = DEFAULT_WHEEL_KD;

        frontLController = new PIDProfile(WHEEL_KP, WHEEL_KI, WHEEL_KD, MAX_WHEEL_ACCELERATION);
        frontRController = new PIDProfile(WHEEL_KP, WHEEL_KI, WHEEL_KD, MAX_WHEEL_ACCELERATION);
        backLController = new PIDProfile(WHEEL_KP, WHEEL_KI, WHEEL_KD, MAX_WHEEL_ACCELERATION);
        backRController = new PIDProfile(WHEEL_KP, WHEEL_KI, WHEEL_KD, MAX_WHEEL_ACCELERATION);

        this.wheelPower = wheelPower;

        //
        // Arm
        //
        arm = opMode.hardwareMap.get(DcMotorEx.class, ARM_STR);

        arm.setDirection(ARM_REVERSED ? DcMotorEx.Direction.REVERSE : DcMotorEx.Direction.FORWARD);

        arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        arm.setTargetPosition(0);

        arm.setTargetPositionTolerance(ARM_POSITION_TOLERANCE);

        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        arm.setPower(0);

        //
        // Lift
        //
        leftLift = opMode.hardwareMap.get(DcMotorEx.class, LEFT_LIFT_STR);
        rightLift = opMode.hardwareMap.get(DcMotorEx.class, RIGHT_LIFT_STR);

        leftLift.setDirection(LEFT_LIFT_REVERSED ? DcMotorEx.Direction.REVERSE : DcMotorEx.Direction.FORWARD);
        rightLift.setDirection(RIGHT_LIFT_REVERSED ? DcMotorEx.Direction.REVERSE : DcMotorEx.Direction.FORWARD);

        leftLift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        leftLift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        leftLift.setTargetPosition(0);
        rightLift.setTargetPosition(0);

        leftLift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightLift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        leftLift.setPower(0);
        rightLift.setPower(0);

        //
        // Claws
        //
        leftClaw = opMode.hardwareMap.get(Servo.class, LEFT_CLAW_STR);
        rightClaw = opMode.hardwareMap.get(Servo.class, RIGHT_CLAW_STR);

        leftClaw.setDirection(LEFT_CLAW_REVERSED ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        rightClaw.setDirection(RIGHT_CLAW_REVERSED ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);

        moveClaws(true, true, ClawPositions.CLAWS_CLOSED);

        //
        // Plane
        //
        plane = opMode.hardwareMap.get(Servo.class, PLANE_STR);

        plane.setDirection(PLANE_REVERSED ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);

        plane.setPosition(PLANE_REST_POS);

        //
        // Systems
        //
        imu = opMode.hardwareMap.get(IMU.class, IMU_STR);
        imu.initialize(IMU_DEFAULT_PARAMS);
        if (resetIMUYaw) imu.resetYaw();
    }

    public Robot(DrivePeriod drivePeriod, boolean resetIMUYaw, double wheelPower, OpMode opMode) {
        //
        // Robot
        //
        setDriveMode(DEFAULT_DRIVE_MODE);
        this.drivePeriod = drivePeriod;
        this.opMode = opMode;

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

        frontL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        switch (drivePeriod) {
            case DRIVER:
            case TEST:
                frontL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                frontR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                backL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                backR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

                frontL.setPower(0);
                frontR.setPower(0);
                backL.setPower(0);
                backR.setPower(0);
                break;
            case AUTONOMOUS:
                frontL.setTargetPosition(0);
                frontR.setTargetPosition(0);
                backL.setTargetPosition(0);
                backR.setTargetPosition(0);

                frontL.setTargetPositionTolerance(WHEELS_POSITION_TOLERANCE);
                frontR.setTargetPositionTolerance(WHEELS_POSITION_TOLERANCE);
                backL.setTargetPositionTolerance(WHEELS_POSITION_TOLERANCE);
                backR.setTargetPositionTolerance(WHEELS_POSITION_TOLERANCE);

                frontL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                frontR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                backL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                backR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

                frontL.setPower(wheelPower);
                frontR.setPower(wheelPower);
                backL.setPower(wheelPower);
                backR.setPower(wheelPower);
        }

        WHEEL_KP = DEFAULT_WHEEL_KP;
        WHEEL_KI = DEFAULT_WHEEL_KI;
        WHEEL_KD = DEFAULT_WHEEL_KD;

        frontLController = new PIDProfile(WHEEL_KP, WHEEL_KI, WHEEL_KD, MAX_WHEEL_ACCELERATION);
        frontRController = new PIDProfile(WHEEL_KP, WHEEL_KI, WHEEL_KD, MAX_WHEEL_ACCELERATION);
        backLController = new PIDProfile(WHEEL_KP, WHEEL_KI, WHEEL_KD, MAX_WHEEL_ACCELERATION);
        backRController = new PIDProfile(WHEEL_KP, WHEEL_KI, WHEEL_KD, MAX_WHEEL_ACCELERATION);

        this.wheelPower = wheelPower;


        //
        // Arm
        //
        arm = opMode.hardwareMap.get(DcMotorEx.class, ARM_STR);

        arm.setDirection(ARM_REVERSED ? DcMotorEx.Direction.REVERSE : DcMotorEx.Direction.FORWARD);

        arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        arm.setTargetPosition(0);

        arm.setTargetPositionTolerance(ARM_POSITION_TOLERANCE);

        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        arm.setPower(0);

        //
        // Lift
        //
        leftLift = opMode.hardwareMap.get(DcMotorEx.class, LEFT_LIFT_STR);
        rightLift = opMode.hardwareMap.get(DcMotorEx.class, RIGHT_LIFT_STR);

        leftLift.setDirection(LEFT_LIFT_REVERSED ? DcMotorEx.Direction.REVERSE : DcMotorEx.Direction.FORWARD);
        rightLift.setDirection(RIGHT_LIFT_REVERSED ? DcMotorEx.Direction.REVERSE : DcMotorEx.Direction.FORWARD);

        leftLift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        leftLift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        leftLift.setTargetPosition(0);
        rightLift.setTargetPosition(0);

        leftLift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightLift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        leftLift.setPower(0);
        rightLift.setPower(0);

        //
        // Claws
        //
        leftClaw = opMode.hardwareMap.get(Servo.class, LEFT_CLAW_STR);
        rightClaw = opMode.hardwareMap.get(Servo.class, RIGHT_CLAW_STR);

        leftClaw.setDirection(LEFT_CLAW_REVERSED ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        rightClaw.setDirection(RIGHT_CLAW_REVERSED ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);

        moveClaws(true, true, ClawPositions.CLAWS_CLOSED);

        //
        // Plane
        //
        plane = opMode.hardwareMap.get(Servo.class, PLANE_STR);

        plane.setDirection(PLANE_REVERSED ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);

        plane.setPosition(PLANE_REST_POS);

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
    public void start() {
        arm.setPower(DEFAULT_ARM_POWER);
        leftLift.setPower(1);
        rightLift.setPower(1);
    }

    public void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public void setDriveMode(DriveMode driveMode) {
        this.driveMode = driveMode;
    }

    public void switchDriveMode() {
        switch (driveMode) {
            case ROBOT:
                setDriveMode(DriveMode.FIELD);
                break;
            case FIELD:
                setDriveMode(DriveMode.ROBOT);
                break;
        }
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
                    case ROBOT:
                        opMode.telemetry.addData("Robot drive mode:", driveMode);
                        break;
                    case WHEELS:
                        opMode.telemetry.addData("FL busy:", frontL.isBusy());
                        opMode.telemetry.addData("FR busy:", frontR.isBusy());
                        opMode.telemetry.addData("BL busy:", backL.isBusy());
                        opMode.telemetry.addData("BR busy:", backR.isBusy());

                        opMode.telemetry.addData("FL power:", frontL.getPower());
                        opMode.telemetry.addData("FR power:", frontR.getPower());
                        opMode.telemetry.addData("BL power:", backL.getPower());
                        opMode.telemetry.addData("BR power:", backR.getPower());
                        break;
                    case ARM:
                        opMode.telemetry.addData("Arm busy:", arm.isBusy());
                        opMode.telemetry.addData("Arm position:", arm.getCurrentPosition());

                        switch (armPosIndex) {
                            case 0:
                                opMode.telemetry.addLine("Arm target position: Rest");
                                break;
                            case 1:
                                opMode.telemetry.addLine("Arm target position: Score");
                                break;
                            case 2:
                                opMode.telemetry.addLine("Arm target position: Load");
                                break;
                        }
                        break;
                    case LIFT:
                        opMode.telemetry.addData("L Lift busy:", leftLift.isBusy());
                        opMode.telemetry.addData("R Lift busy:", rightLift.isBusy());
                        opMode.telemetry.addData("L Lift power:", leftLift.getPower());
                        opMode.telemetry.addData("R Lift power:", rightLift.getPower());
                        switch (currLiftPosition) {
                            case UP:
                                opMode.telemetry.addLine("Lift direction: Up");
                                break;
                            case HANG:
                                opMode.telemetry.addLine("Lift direction: Hang");
                                break;
                        }
                        break;
                    case CLAWS:
                        opMode.telemetry.addData("LeftClaw Position:", leftClaw.getPosition());
                        opMode.telemetry.addData("RightClaw Position:", rightClaw.getPosition());
                        break;
                    case IMU:
                        opMode.telemetry.addData("IMU angle:", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
                        break;
                }
            }
        } else
            update(Systems.ROBOT, Systems.WHEELS, Systems.ARM, Systems.LIFT, Systems.CLAWS, Systems.IMU);
        opMode.telemetry.update();
    }

    //
    // Wheels
    //
    public void positionDrive(int frontLRot, int frontRRot, int backLRot, int backRRot) {
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
     * <p>
     * On field mode the movement is based on the direction of the gyroscope, while on robot mode it's based on the robot direction
     * <p>
     * Uses PID Control and sends velocity to the FTC Dashboard if the drive period is set to TEST
     *
     * @param forwardPower  The multiplier of the movement in the forward/backward direction. Ranges from -1 to 1
     * @param sidePower     The multiplier of the movement in the left/right direction. Ranges from -1 to 1
     * @param rotationPower The multiplier of the turning. Ranges from -1 to 1
     */
    public void drive(double forwardPower, double sidePower, double rotationPower) {

        // double denominator = Math.max(Math.abs(forwardPower) + Math.abs(sidePower) + Math.abs(sidePower), 1);

        double frontLPower = 0;
        double frontRPower = 0;
        double backLPower = 0;
        double backRPower = 0;

        double frontLTargetVelocity = 0;
        double frontRTargetVelocity = 0;
        double backLTargetVelocity = 0;
        double backRTargetVelocity = 0;

        rotationPower *= ROTATION_POWER_MULTIPLIER;

        switch (driveMode) {
            case ROBOT:
                frontLTargetVelocity = forwardPower + sidePower + rotationPower;
                frontRTargetVelocity = forwardPower + sidePower + rotationPower;
                backLTargetVelocity = forwardPower + sidePower + rotationPower;
                backRTargetVelocity = forwardPower + sidePower + rotationPower;

                frontLPower = frontLController.getPower(frontLTargetVelocity * MAX_WHEEL_VELOCITY, frontL.getVelocity());
                frontRPower = frontRController.getPower(frontRTargetVelocity * MAX_WHEEL_VELOCITY, frontR.getVelocity());
                backLPower = backLController.getPower(backLTargetVelocity * MAX_WHEEL_VELOCITY, backL.getVelocity());
                backRPower = backRController.getPower(backRTargetVelocity * MAX_WHEEL_VELOCITY, backR.getVelocity());

                break;
            case FIELD:
                double robotRotation = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                double forwardField = forwardPower * Math.cos(robotRotation) - sidePower * Math.sin(robotRotation);
                double sideField = forwardPower * Math.sin(robotRotation) + sidePower * Math.cos(robotRotation);
                sideField *= SIDE_POWER_PERFECTION_MULTIPLIER;

                frontLTargetVelocity = forwardField + sideField + rotationPower;
                frontRTargetVelocity = forwardField + sideField + rotationPower;
                backLTargetVelocity = forwardField + sideField + rotationPower;
                backRTargetVelocity = forwardField + sideField + rotationPower;

                frontLPower = frontLController.getPower(frontLTargetVelocity * MAX_WHEEL_VELOCITY, frontL.getVelocity());
                frontRPower = frontRController.getPower(frontRTargetVelocity * MAX_WHEEL_VELOCITY, frontR.getVelocity());
                backLPower = backLController.getPower(backLTargetVelocity * MAX_WHEEL_VELOCITY, backL.getVelocity());
                backRPower = backRController.getPower(backRTargetVelocity * MAX_WHEEL_VELOCITY, backR.getVelocity());

                break;
        }

        if (drivePeriod == DrivePeriod.TEST) {
            frontLController.updateCoefficients(WHEEL_KP, WHEEL_KI, WHEEL_KD);
            frontRController.updateCoefficients(WHEEL_KP, WHEEL_KI, WHEEL_KD);
            backLController.updateCoefficients(WHEEL_KP, WHEEL_KI, WHEEL_KD);
            backRController.updateCoefficients(WHEEL_KP, WHEEL_KI, WHEEL_KD);

            FtcDashboard dashboard = FtcDashboard.getInstance();
            Telemetry telemetry = dashboard.getTelemetry();

            telemetry.addData("FrontL target velocity", frontLTargetVelocity);
            telemetry.addData("FrontL current velocity", frontL.getVelocity());
            telemetry.addData("FrontL current velocity", frontL.getVelocity());
            telemetry.addData("FrontR target velocity", frontRTargetVelocity);
            telemetry.addData("FrontR current velocity", frontR.getVelocity());
            telemetry.addData("FrontR current velocity", frontR.getVelocity());
            telemetry.addData("backL target velocity", backLTargetVelocity);
            telemetry.addData("BackL current velocity", backL.getVelocity());
            telemetry.addData("BackL current velocity", backL.getVelocity());
            telemetry.addData("BackR target velocity", backRTargetVelocity);
            telemetry.addData("BackR current velocity", backR.getVelocity());
            telemetry.addData("BackR current velocity", backR.getVelocity());

            telemetry.update();
        }

        frontL.setPower(frontLPower * wheelPower);
        frontR.setPower(frontRPower * wheelPower);
        backL.setPower(backLPower * wheelPower);
        backR.setPower(backRPower * wheelPower);
    }

    public void moveForward(int rotation, @Nullable boolean... individualWait) {
        int frontLTargetPos = frontL.getCurrentPosition() + rotation;
        int frontRTargetPos = frontR.getCurrentPosition() + rotation;
        int backLTargetPos = backL.getCurrentPosition() + rotation;
        int backRTargetPos = backR.getCurrentPosition() + rotation;

        frontL.setTargetPosition(frontLTargetPos);
        frontR.setTargetPosition(frontRTargetPos);
        backL.setTargetPosition(backLTargetPos);
        backR.setTargetPosition(backRTargetPos);

        if (individualWait != null && individualWait.length > 0 && individualWait[0]) {
            waitForSystem(50, Systems.WHEELS);
        }
    }

    public void moveBackward(int rotation, @Nullable boolean... individualWait) {
        int frontLTargetPos = frontL.getCurrentPosition() - rotation;
        int frontRTargetPos = frontR.getCurrentPosition() - rotation;
        int backLTargetPos = backL.getCurrentPosition() - rotation;
        int backRTargetPos = backR.getCurrentPosition() - rotation;

        frontL.setTargetPosition(frontLTargetPos);
        frontR.setTargetPosition(frontRTargetPos);
        backL.setTargetPosition(backLTargetPos);
        backR.setTargetPosition(backRTargetPos);

        if (individualWait != null && individualWait.length > 0 && individualWait[0]) {
            waitForSystem(50, Systems.WHEELS);
        }
    }

    public void moveLeft(int rotation, @Nullable boolean... individualWait) {
        int frontLTargetPos = frontL.getCurrentPosition() - rotation;
        int frontRTargetPos = frontR.getCurrentPosition() + rotation;
        int backLTargetPos = backL.getCurrentPosition() + rotation;
        int backRTargetPos = backR.getCurrentPosition() - rotation;

        frontL.setTargetPosition(frontLTargetPos);
        frontR.setTargetPosition(frontRTargetPos);
        backL.setTargetPosition(backLTargetPos);
        backR.setTargetPosition(backRTargetPos);

        if (individualWait != null && individualWait.length > 0 && individualWait[0]) {
            waitForSystem(50, Systems.WHEELS);
        }
    }

    public void moveRight(int rotation, @Nullable boolean... individualWait) {
        int frontLTargetPos = frontL.getCurrentPosition() + rotation;
        int frontRTargetPos = frontR.getCurrentPosition() - rotation;
        int backLTargetPos = backL.getCurrentPosition() - rotation;
        int backRTargetPos = backR.getCurrentPosition() + rotation;

        frontL.setTargetPosition(frontLTargetPos);
        frontR.setTargetPosition(frontRTargetPos);
        backL.setTargetPosition(backLTargetPos);
        backR.setTargetPosition(backRTargetPos);

        if (individualWait != null && individualWait.length > 0 && individualWait[0]) {
            waitForSystem(50, Systems.WHEELS);
        }
    }

    public void turnLeft(int rotation, @Nullable boolean... individualWait) {
        int frontLTargetPos = frontL.getCurrentPosition() - rotation;
        int frontRTargetPos = frontR.getCurrentPosition() + rotation;
        int backLTargetPos = backL.getCurrentPosition() - rotation;
        int backRTargetPos = backR.getCurrentPosition() + rotation;

        frontL.setTargetPosition(frontLTargetPos);
        frontR.setTargetPosition(frontRTargetPos);
        backL.setTargetPosition(backLTargetPos);
        backR.setTargetPosition(backRTargetPos);

        if (individualWait != null && individualWait.length > 0 && individualWait[0]) {
            waitForSystem(50, Systems.WHEELS);
        }
    }

    public void turnRight(int rotation, @Nullable boolean... individualWait) {
        int frontLTargetPos = frontL.getCurrentPosition() + rotation;
        int frontRTargetPos = frontR.getCurrentPosition() - rotation;
        int backLTargetPos = backL.getCurrentPosition() + rotation;
        int backRTargetPos = backR.getCurrentPosition() - rotation;

        frontL.setTargetPosition(frontLTargetPos);
        frontR.setTargetPosition(frontRTargetPos);
        backL.setTargetPosition(backLTargetPos);
        backR.setTargetPosition(backRTargetPos);

        if (individualWait != null && individualWait.length > 0 && individualWait[0]) {
            waitForSystem(50, Systems.WHEELS);
        }
    }

    public void turnAngleLeft(double angle) {
        double imuAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        if (angle > 180) {
            currImuTargetAngle = -180 + (angle % 180);
        } else currImuTargetAngle = angle;

        while (imuAngle > currImuTargetAngle + IMU_TOLERANCE_DEGREES || imuAngle < currImuTargetAngle - IMU_TOLERANCE_DEGREES) {
            turnLeft(50);

            imuAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        }

        stopWheels();

        sleep(500);
    }

    public void turnAngleRight(double angle) {
        double imuAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        if (angle < -180) {
            currImuTargetAngle = 180 - (Math.abs(angle) % 180);
        } else currImuTargetAngle = angle;

        while (imuAngle > currImuTargetAngle + IMU_TOLERANCE_DEGREES || imuAngle < currImuTargetAngle - IMU_TOLERANCE_DEGREES) {
            turnRight(50);

            imuAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        }

        stopWheels();

        sleep(500);
    }

    public void moveDirection(double angle, int time) {
        ElapsedTime timer = new ElapsedTime();
        double startTime = timer.seconds();

        double forwardPower = Math.cos(Math.toRadians(angle));
        double sidePower = Math.sin(Math.toRadians(angle));

        while (startTime + time > timer.seconds()) {
            double robotRotation = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double forwardField = forwardPower * Math.cos(robotRotation) - sidePower * Math.sin(robotRotation);
            double sideField = forwardPower * Math.sin(robotRotation) + sidePower * Math.cos(robotRotation);
            sideField *= SIDE_POWER_PERFECTION_MULTIPLIER;

            double frontLPower = forwardField + sideField; // + rotationPower
            double frontRPower = forwardField - sideField; // - rotationPower
            double backLPower = forwardField - sideField; // + rotationPower
            double backRPower = forwardField + sideField; // - rotationPower

            frontL.setPower(frontLPower * wheelPower);
            frontR.setPower(frontRPower * wheelPower);
            backL.setPower(backLPower * wheelPower);
            backR.setPower(backRPower * wheelPower);
        }
    }

    public void setWheelPower(double wheelPower) {
        this.wheelPower = wheelPower;

        if (drivePeriod == DrivePeriod.AUTONOMOUS) {
            frontL.setPower(wheelPower);
            frontR.setPower(wheelPower);
            backL.setPower(wheelPower);
            backR.setPower(wheelPower);
        }
    }

    public void stopWheels() {
        frontL.setTargetPosition(frontL.getCurrentPosition());
        frontR.setTargetPosition(frontR.getCurrentPosition());
        backL.setTargetPosition(backL.getCurrentPosition());
        backR.setTargetPosition(backR.getCurrentPosition());
    }

    public void waitForWheels() {
        while (frontL.isBusy() || frontR.isBusy() || backL.isBusy() || backR.isBusy()) {
            sleep(1);
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
            waitForSystem(50, Systems.ARM);
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
            sleep(1);
        }
    }

    //
    // Lift
    //
    public void moveLift() {
        switch (currLiftPosition) {
            case UP:
                leftLift.setTargetPosition(LIFT_UP_POSITION);
                rightLift.setTargetPosition(LIFT_UP_POSITION);
                break;
            case HANG:
                leftLift.setTargetPosition(LIFT_HANGING_POSITION);
                rightLift.setTargetPosition(LIFT_HANGING_POSITION);
                break;
        }
    }

    public void switchLiftPosition() {
        switch (currLiftPosition) {
            case UP:
                currLiftPosition = LiftPositions.HANG;
                break;
            case HANG:
                currLiftPosition = LiftPositions.UP;
                break;
        }
    }

    public void waitForLift() {
        while (leftLift.isBusy() || rightLift.isBusy()) {
            sleep(1);
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

        if (individualWait != null && individualWait.length > 0 && individualWait[0]) {
            waitForSystem(50, Systems.CLAWS);
        }
    }

    public void waitForClaws() {
        sleep(500);
    }

    //
    // Plane
    //
    public void launchPlane() {
        plane.setPosition(PLANE_LAUNCH_POS);
    }

    //
    // Systems
    //
    /*
    private void waitForImu() {
    }*/

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
                    break;/*
                case IMU:
                    waitForImu();
                    break;*/
                default:
                    waitForSystem(50, Systems.WHEELS, Systems.ARM, Systems.LIFT, Systems.CLAWS);
            }
        }

        sleep(extraTime);
    }
}
