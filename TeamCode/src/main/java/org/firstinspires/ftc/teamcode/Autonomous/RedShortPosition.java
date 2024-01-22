package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Robot.*;
import static org.firstinspires.ftc.teamcode.RobotParameters.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot;


@Autonomous(name = "Red Short Pos AS")
public class RedShortPosition extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot robot =  new Robot();

        waitForStart();

        frontL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.moveClaws(true, true, CLAWS_CLOSED);

        waitForStart();

        // Move forward
        robot.positionDrive(1360, 1360, 1360, 1360);
        robot.waitForSystem(WHEELS_SYSTEM_STR);
        sleep(20);

        // Move backward
        robot.positionDrive(-360, -360, -360, -360);
        robot.waitForSystem(WHEELS_SYSTEM_STR);
        sleep(20);

        // Turn right
        robot.positionDrive(-900, 900, 900, -900);
        robot.waitForSystem(WHEELS_SYSTEM_STR);
        sleep(20);

        // Move forward
        robot.positionDrive(1550, 1550, 1550, 1550);
        robot.waitForSystem(WHEELS_SYSTEM_STR);
        sleep(20);

        // Move arm to placing position
        robot.moveArm(ARM_SCORE_BACKDROP_POS);
        robot.waitForSystem(ARM_SYSTEM_STR);
        sleep(500);

        // Open claws
        robot.moveClaws(true, true, CLAWS_FALL);
        robot.waitForSystem(CLAWS_SYSTEM_STR);
        sleep(500);

        // Move arm to rest position
        robot.moveArm(ARM_REST_POSITION);
        robot.waitForSystem(ARM_SYSTEM_STR);
        sleep(500);

        // Move forward and close claws
        robot.positionDrive(-150, -150, -150, -150);
        robot.moveClaws(true, true, CLAWS_CLOSED);
        robot.waitForSystem(WHEELS_SYSTEM_STR, CLAWS_SYSTEM_STR);
        sleep(20);

        // Crabwalk left
        robot.positionDrive(-1600, 1600, 1600, -1600);
        robot.waitForSystem(WHEELS_SYSTEM_STR);
        sleep(20);

        // Move forward
        robot.positionDrive(700, 700, 700, 700);
        robot.waitForSystem(WHEELS_SYSTEM_STR);
    }
}