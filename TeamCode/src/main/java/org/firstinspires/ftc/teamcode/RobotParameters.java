package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

public class RobotParameters {
    //
    // Robot
    //
    public static enum DriveMode {
        FIELD, ROBOT
    }
    public static final DriveMode DEFAULT_DRIVE_MODE = DriveMode.FIELD;

    //
    // Wheels
    //
    public static final String FRONT_LEFT_STR              = "frontLeft";
    public static final String FRONT_RIGHT_STR             = "frontRight";
    public static final String BACK_LEFT_STR               = "backLeft";
    public static final String BACK_RIGHT_STR              = "backRight";
    public static final boolean FRONT_LEFT_REVERSED        = true;
    public static final boolean FRONT_RIGHT_REVERSED       = false;
    public static final boolean BACK_LEFT_REVERSED         = true;
    public static final boolean BACK_RIGHT_REVERSED        = false;
    public static final double WHEEL_PPR                   = ((1+((double) 46 /17))) * (1+((double) 46 /11)) * 28;
    public static final int WHEEL_RPM                      = 312;
    public static final double WHEEL_RPS                   = (double) WHEEL_RPM / 60;
    public static final double WHEEL_DEGREES_PER_SECOND    = WHEEL_RPS * 360;
    public static final double WHEEL_PPM                   = WHEEL_PPR * WHEEL_RPM;
    public static final double WHEEL_PPS                   = WHEEL_PPM * 60;
    public static final double ROTATION_POWER_MULTIPLIER   = (double) 1 / 3;

    //
    // Arm
    //
    public static final String ARM_STR                     = "arm";
    public static final boolean ARM_REVERSED               = true;
    public static final double DEFAULT_ARM_POWER           = 0.4;
    public static final int ARM_LOAD_POS                   = -1800;
    public static final int ARM_REST_POS                   = -20;
    public static final int ARM_SCORE_POS                  = -1100;
    public static int[] ArmPositions                       = new int[]{
      ARM_REST_POS, ARM_SCORE_POS, ARM_LOAD_POS
    };

    //
    // Lift
    //
    public static final String LIFT_STR                    = "lift";
    public static final boolean LIFT_REVERSED              = false;
    public static enum LiftDirections{
        UP, DOWN, REST
    }

    //
    // Claws
    //
    public static final String LEFT_CLAW_STR               = "leftClaw";
    public static final String RIGHT_CLAW_STR              = "rightClaw";
    public static final boolean LEFT_CLAW_REVERSED         = true;
    public static final boolean RIGHT_CLAW_REVERSED        = false;
    public static final double LEFT_CLAW_CLOSED_POS        = 0.75;
    public static final double LEFT_CLAW_OPEN_POS          = 0.5;
    public static final double LEFT_CLAW_FALL_POS          = 0.1;
    public static final double RIGHT_CLAW_CLOSED_POS       = 0.75;
    public static final double RIGHT_CLAW_OPEN_POS         = 0.5;
    public static final double RIGHT_CLAW_FALL_POS         = 0.1;
    public static final int CLAWS_CLOSED                   = 0;
    public static final int CLAWS_OPEN                     = 1;
    public static final int CLAWS_FALL                     = 2;
    public static enum ClawPositions {
        CLAWS_CLOSED, CLAWS_OPEN, CLAWS_FALL;
        public static double leftClaw(ClawPositions clawPos){
            double targetPos = 0;
            switch(clawPos){
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
        };
        public static double rightClaw(ClawPositions clawPos){
            double targetPos = 0;
            switch(clawPos){
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
        };
    }

    //
    // Systems
    //
    public static enum Systems {
        WHEELS, ARM, CLAWS, LIFT, IMU
    }
    public static final String IMU_STR                     = "imu";
    public static final RevHubOrientationOnRobot.LogoFacingDirection LOGO_DIRECTION = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    public static final RevHubOrientationOnRobot.UsbFacingDirection USB_DIRECTION = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
    public static RevHubOrientationOnRobot ORIENTATION_ON_ROBOT= new RevHubOrientationOnRobot(LOGO_DIRECTION, USB_DIRECTION);
    public static IMU.Parameters IMU_DEFAULT_PARAMS  = new IMU.Parameters(ORIENTATION_ON_ROBOT);
}

