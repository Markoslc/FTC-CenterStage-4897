package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Orientation.Position2D;

public class RobotParameters {
    //
    // Robot
    //
    public enum DriveMode {
        FIELD, ROBOT
    }

    public enum DrivePeriod {
        AUTONOMOUS, DRIVER
    }

    //
    // Gamepad
    //
    public static final boolean L_STICK_Y_REVERSED = true;
    public static final boolean L_STICK_X_REVERSED = false;
    public static final boolean R_STICK_Y_REVERSED = true;
    public static final boolean R_STICK_X_REVERSED = false;

    public enum GamepadButtons {
        A, B, X, Y,
        DPAD_UP, DPAD_DOWN, DPAD_LEFT, DPAD_RIGHT,
        L_BUMPER, R_BUMPER,
        BACK, START, GUIDE,
        L_STICK_BUTTON, R_STICK_BUTTON
    }

    public enum GamepadTriggers {
        L_TRIGGER, R_TRIGGER
    }

    public enum GamepadSticks {
        L_STICK, R_STICK
    }

    public static final DriveMode DEFAULT_DRIVE_MODE = DriveMode.FIELD;

    //
    // Wheels
    //
    public static final String  FRONT_LEFT_STR                   = "frontLeft";
    public static final String  FRONT_RIGHT_STR                  = "frontRight";
    public static final String  BACK_LEFT_STR                    = "backLeft";
    public static final String  BACK_RIGHT_STR                   = "backRight";
    public static final boolean FRONT_LEFT_REVERSED              = true;
    public static final boolean FRONT_RIGHT_REVERSED             = false;
    public static final boolean BACK_LEFT_REVERSED               = false;
    public static final boolean BACK_RIGHT_REVERSED              = true;
    public static final double  WHEEL_PPR                        = ((1 + ((double) 46 / 17))) * (1 + ((double) 46 / 11)) * 28;
    public static final int     WHEEL_RPM                        = 312;
    public static final double  WHEEL_RPS                        = (double) WHEEL_RPM / 60;
    public static final double  WHEEL_DEGREES_PER_SECOND         = WHEEL_RPS * 360;
    public static final double  WHEEL_PPM                        = WHEEL_PPR * WHEEL_RPM;
    public static final double  WHEEL_PPS                        = WHEEL_PPM * 60;
    public static final double  ROTATION_POWER_MULTIPLIER        = (double) 1 / 3;
    public static final double  SIDE_POWER_PERFECTION_MULTIPLIER = 1.1;
    public static final int     WHEELS_POSITION_TOLERANCE        = 5;

    //
    // Arm
    //
    public static final String  ARM_STR                = "arm";
    public static final boolean ARM_REVERSED           = false;
    public static final double  DEFAULT_ARM_POWER      = 0.4;
    public static final int     ARM_LOAD_POS           = -1900;
    public static final int     ARM_REST_POS           = 0;
    public static final int     ARM_SCORE_POS          = -1100;
    public static final int[]   ArmPositions           = new int[]{
            ARM_REST_POS, ARM_SCORE_POS, ARM_LOAD_POS
    };
    public static final int     ARM_POSITION_TOLERANCE = 2;

    //
    // Lift
    //
    public static final String  LIFT_STR      = "lift";
    public static final boolean LIFT_REVERSED = true;

    public enum LiftDirections {
        UP, DOWN
    }

    //
    // Claws
    //
    public static final  String  LEFT_CLAW_STR         = "leftClaw";
    public static final  String  RIGHT_CLAW_STR        = "rightClaw";
    public static final  boolean LEFT_CLAW_REVERSED    = true;
    public static final  boolean RIGHT_CLAW_REVERSED   = false;
    private static final double  LEFT_CLAW_CLOSED_POS  = 0.9;
    private static final double  LEFT_CLAW_OPEN_POS    = 0.5;
    private static final double  LEFT_CLAW_FALL_POS    = 0.1;
    private static final double  RIGHT_CLAW_CLOSED_POS = 0.725;
    private static final double  RIGHT_CLAW_OPEN_POS   = 0.5;
    private static final double  RIGHT_CLAW_FALL_POS   = 0.1;

    public enum ClawPositions {
        CLAWS_CLOSED, CLAWS_OPEN, CLAWS_FALL;

        public static double leftClaw(ClawPositions clawPos) {
            double targetPos = 0;
            switch (clawPos) {
                case CLAWS_CLOSED:
                    targetPos = LEFT_CLAW_CLOSED_POS;
                    break;
                case CLAWS_OPEN:
                    targetPos = LEFT_CLAW_OPEN_POS;
                    break;
                case CLAWS_FALL:
                    targetPos = LEFT_CLAW_FALL_POS;
                    break;
            }
            return targetPos;
        }

        public static double rightClaw(ClawPositions clawPos) {
            double targetPos = 0;
            switch (clawPos) {
                case CLAWS_CLOSED:
                    targetPos = RIGHT_CLAW_CLOSED_POS;
                    break;
                case CLAWS_OPEN:
                    targetPos = RIGHT_CLAW_OPEN_POS;
                    break;
                case CLAWS_FALL:
                    targetPos = RIGHT_CLAW_FALL_POS;
                    break;
            }
            return targetPos;
        }
    }

    //
    // Plane
    //
    public static final String  PLANE_STR        = "plane";
    public static final boolean PLANE_REVERSED   = false;
    public static final double  PLANE_REST_POS   = 1;
    public static final double  PLANE_LAUNCH_POS = 0.5;

    //
    // Systems
    //
    public enum Systems {
        ROBOT, WHEELS, ARM, CLAWS, LIFT, IMU, PLANE, CAMERA
    }

    public static final String                                       IMU_STR                      = "imu";
    public static final double                                       IMU_TOLERANCE_DEGREES        = 2;
    public static final RevHubOrientationOnRobot.LogoFacingDirection LOGO_DIRECTION               = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    public static final RevHubOrientationOnRobot.UsbFacingDirection  USB_DIRECTION                = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
    public static       RevHubOrientationOnRobot                     DEFAULT_ORIENTATION_ON_ROBOT = new RevHubOrientationOnRobot(LOGO_DIRECTION, USB_DIRECTION);
    public static       IMU.Parameters                               IMU_DEFAULT_PARAMS           = new IMU.Parameters(DEFAULT_ORIENTATION_ON_ROBOT);
    public static final int                                          VERBOSE                      = 4;
    /**
     * 0: NOTHING
     * 1: ERRORS ONLY
     * 2: WARNINGS AND MALFUNCTIONS
     * 3: INFORMATION
     * 4: UNIMPORTANT INFORMATION
     */

    //
    //Position Settings
    //
    public static       double                                       POSITION_TOLERANCE           = 0.1;
    public static enum Alliance {
        RED_ALLIANCE, BLUE_ALLIANCE
    }
    public static final Position2D[]                                 APRIL_TAG_POSES              = new Position2D[]{
            new Position2D(29.845, 77.7, 270.0),    // TagId 1
            new Position2D(29.845, 92.94, 270.0),   // TagId 2
            new Position2D(29.845, 108.18, 270.0),  // TagId 3
            new Position2D(29.845, 257.58, 270.0),  // TagId 4
            new Position2D(29.845, 272.82, 270.0),  // TagId 5
            new Position2D(29.845, 288.1, 270.0),   // TagId 6
            new Position2D(365.76, 286.08, 90.0), // TagId 7
            new Position2D(365.76, 272.11, 90.0), // TagId 8
            new Position2D(365.76, 93.65, 90.0),  // TagId 9
            new Position2D(365.76, 79.68, 90.0)   // TagId 10
    };


    //
    // Camera
    //
    public static final String  CAMERA_STR       = "Webcam 1";
    public static final int     CAMERA_WIDTH     = 1280;
    public static final int     CAMERA_HEIGHT    = 720;
    public static final boolean USE_WEBCAM       = true;
    public static final boolean ENABLE_LIVE_VIEW = true;
    public static enum TeamPropPosition {
        LEFT, CENTER, RIGHT
    }


}

