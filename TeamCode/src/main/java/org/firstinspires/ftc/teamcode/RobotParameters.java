package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

public class RobotParameters {
    // Robot
    public static enum DriveMode {
        FIELD, ROBOT
    }
    public static final DriveMode DEFAULT_DRIVE_MODE = DriveMode.FIELD;
    // Wheelsfinal
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
    public static final double WHEEL_PPM                   = WHEEL_PPR * WHEEL_RPM;
    public static final double WHEEL_PPS                  = WHEEL_PPM * 60;

    // Arm
    public static final String ARM_STR                     = "arm";
    public static final boolean ARM_REVERSED               = true;
    public static final int ARM_LOAD_POS             = -1800;
    public static final int ARM_REST_POSITION        = -20;
    public static final int ARM_SCORE_BACKDROP_POS   = -1100;

    // Claws
    public static final String LEFT_CLAW_STR               = "leftClaw";
    public static final String RIGHT_CLAW_STR              = "rightClaw";
    public static final boolean LEFT_CLAW_REVERSED         = true;
    public static final boolean RIGHT_CLAW_REVERSED        = false;
    public static final double LEFT_CLAW_CLOSED_POS        = 0.75;
    public static final double LEFT_CLAW_OPEN_POS          = 0.5;
    public static final double LEFT_CLAW_FALL_POS          = 0.1;
    public static final double[] LEFT_CLAW_POS = new double[]{
            LEFT_CLAW_CLOSED_POS, LEFT_CLAW_OPEN_POS, LEFT_CLAW_FALL_POS
    };
    public static final double RIGHT_CLAW_CLOSED_POS       = 0.75;
    public static final double RIGHT_CLAW_OPEN_POS         = 0.5;
    public static final double RIGHT_CLAW_FALL_POS         = 0.1;
    public static final double[] RIGHT_CLAW_POS = new double[]{
            RIGHT_CLAW_CLOSED_POS, RIGHT_CLAW_OPEN_POS, RIGHT_CLAW_FALL_POS
    };
    public static final int CLAWS_CLOSED                   = 0;
    public static final int CLAWS_OPEN                     = 1;
    public static final int CLAWS_FALL                     = 2;

    // Systems
    public static enum systems{
        WHEELS, ARM, CLAWS
    }
    public static String IMU_STR                     = "imu";
    public static RevHubOrientationOnRobot.LogoFacingDirection LOGO_DIRECTION = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    public static RevHubOrientationOnRobot.UsbFacingDirection USB_DIRECTION = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
    public static RevHubOrientationOnRobot ORIENTATION_ON_ROBOT= new RevHubOrientationOnRobot(LOGO_DIRECTION, USB_DIRECTION);
    public static IMU.Parameters IMU_DEFAULT_PARAMS  = new IMU.Parameters(ORIENTATION_ON_ROBOT);
}

