package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Robot.backL;
import static org.firstinspires.ftc.teamcode.Robot.backR;
import static org.firstinspires.ftc.teamcode.Robot.frontL;
import static org.firstinspires.ftc.teamcode.Robot.frontR;
import static org.firstinspires.ftc.teamcode.RobotParameters.ARM_REST_POSITION;
import static org.firstinspires.ftc.teamcode.RobotParameters.ARM_SCORE_BACKDROP_POS;
import static org.firstinspires.ftc.teamcode.RobotParameters.CLAWS_CLOSED;
import static org.firstinspires.ftc.teamcode.RobotParameters.CLAWS_FALL;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot;


@Autonomous(name = "Blue Short Pos AS")
public class BlueShortPosition extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot robot =  new Robot();

        waitForStart();


        frontL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sleep(10);

        frontL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.moveClaws(true, true, CLAWS_CLOSED);

        waitForStart();

        // Move forward
        robot.positionDrive(1360, 1360, 1360, 1360);
        sleep(20);

        // Move backward
        robot.positionDrive(-360, -360, -360, -360);
        sleep(20);

        // Turn left
        robot.positionDrive(900, -900, -900, 900);

        sleep(20);

        // Move forward
        robot.positionDrive(1550, 1550, 1550, 1550);
        sleep(20);

        // Move arm to placing position
        robot.moveArm(ARM_SCORE_BACKDROP_POS);
        sleep(500);

        // Open claws
        robot.moveClaws(true, true, CLAWS_FALL);
        sleep(500);

        // Move arm to rest position
        robot.moveArm(ARM_REST_POSITION);
        sleep(1600);

        // Close claws
        robot.moveClaws(true, true, CLAWS_CLOSED);
        sleep(20);

        // Move backward
        robot.positionDrive(-150, -150, -150, -150);
        sleep(20);

        // Crabwalk right
        robot.positionDrive(1600, -1600, -1600, 1600);
        sleep(20);

        // Move forward
        robot.positionDrive(700, 700, 700, 700);
    }
}