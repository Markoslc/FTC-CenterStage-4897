package org.firstinspires.ftc.teamcode;

public class RobotParameters {
    // Wheels
    public static String FRONT_LEFT_STR              = "frontLeft";
    public static String FRONT_RIGHT_STR             = "frontRight";
    public static String BACK_LEFT_STR               = "backLeft";
    public static String BACK_RIGHT_STR              = "backRight";
    public static String WHEELS_SYSTEM_STR           = "Wheels";
    public static boolean FRONT_LEFT_REVERSED        = true;
    public static boolean FRONT_RIGHT_REVERSED       = false;
    public static boolean BACK_LEFT_REVERSED         = true;
    public static boolean BACK_RIGHT_REVERSED        = false;

    // Arm
    public static String ARM_STR                     = "arm";
    public static String ARM_SYSTEM_STR             = "Arm";
    public static boolean ARM_REVERSED               = true;
    public static final int ARM_LOAD_POS             = -1800;
    public static final int ARM_REST_POSITION        = -20;
    public static final int ARM_SCORE_BACKDROP_POS   = -1100;

    // Claws
    public static String LEFT_CLAW_STR               = "leftClaw";
    public static String RIGHT_CLAW_STR              = "rightClaw";
    public static String CLAWS_SYSTEM_STR            = "Claws";
    public static boolean LEFT_CLAW_REVERSED         = true;
    public static boolean RIGHT_CLAW_REVERSED        = false;
    public static double LEFT_CLAW_CLOSED_POS        = 0.75;
    public static double LEFT_CLAW_OPEN_POS          = 0.5;
    public static double LEFT_CLAW_FALL_POS          = 0.1;
    public static double[] LEFT_CLAW_POS = new double[]{
            LEFT_CLAW_CLOSED_POS, LEFT_CLAW_OPEN_POS, LEFT_CLAW_FALL_POS
    };
    public static double RIGHT_CLAW_CLOSED_POS       = 0.75;
    public static double RIGHT_CLAW_OPEN_POS         = 0.5;
    public static double RIGHT_CLAW_FALL_POS         = 0.1;
    public static double[] RIGHT_CLAW_POS = new double[]{
            RIGHT_CLAW_CLOSED_POS, RIGHT_CLAW_OPEN_POS, RIGHT_CLAW_FALL_POS
    };
    public static int CLAWS_CLOSED                   = 0;
    public static int CLAWS_OPEN                     = 1;
    public static int CLAWS_FALL                     = 2;
}

