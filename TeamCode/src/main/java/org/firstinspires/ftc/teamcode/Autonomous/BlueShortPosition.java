package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Robot.*;
import static org.firstinspires.ftc.teamcode.RobotParameters.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot;


@Autonomous(name = "Blue Short Auto AS")
public class BlueShortPosition extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot robot =  new Robot(true, 0.4);

        frontL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.moveClaws(true, true, ClawPositions.CLAWS_CLOSED);

        waitForStart();

        // Move forward
        robot.moveForward(1360);
        robot.waitForSystem(20, Systems.WHEELS);

        // Move backward
        robot.moveBackward(360);
        robot.waitForSystem(20, Systems.WHEELS);

        // Turn left
        robot.turnLeft(900);
        robot.waitForSystem(20, Systems.WHEELS);

        // Move forward
        robot.moveForward(1550);
        robot.waitForSystem(20, Systems.WHEELS);

        // Move arm to placing position
        robot.moveArm(ARM_SCORE_BACKDROP_POS);
        robot.waitForSystem(500, Systems.ARM);

        // Open claws
        robot.moveClaws(true, true, ClawPositions.CLAWS_FALL);
        robot.waitForSystem(500, Systems.CLAWS);

        // Move arm to rest position
        robot.moveArm(ARM_REST_POSITION);
        robot.waitForSystem(500, Systems.ARM);

        // Move backward and close claws
        robot.moveBackward(150);
        robot.moveClaws(true, true, ClawPositions.CLAWS_CLOSED);
        robot.waitForSystem(20, Systems.WHEELS, Systems.CLAWS);

        // Crabwalk right
        robot.moveRight(1600);
        robot.waitForSystem(20, Systems.WHEELS);

        // Move forward
        robot.moveForward(700);
        robot.waitForSystem(0, Systems.WHEELS);
    }
}