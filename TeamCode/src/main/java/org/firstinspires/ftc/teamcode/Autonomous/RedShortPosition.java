package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Robot.*;
import static org.firstinspires.ftc.teamcode.RobotParameters.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot;


@Autonomous(name = "Red Short Auto AS")
public class RedShortPosition extends LinearOpMode {

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

        robot.moveClaws(true, true, CLAWS_CLOSED);

        waitForStart();

        // Move forward
        robot.moveForward(1360);
        robot.waitForSystem(20, systems.WHEELS);

        // Move backward
        robot.moveBackward(360);
        robot.waitForSystem(20, systems.WHEELS);

        // Turn right
        robot.turnRight(900);
        robot.waitForSystem(20, systems.WHEELS);

        // Move forward
        robot.moveForward(1550);
        robot.waitForSystem(20, systems.WHEELS);

        // Move arm to placing position
        robot.moveArm(ARM_SCORE_BACKDROP_POS);
        robot.waitForSystem(500, systems.ARM);

        // Open claws
        robot.moveClaws(true, true, CLAWS_FALL);
        robot.waitForSystem(500, systems.CLAWS);

        // Move arm to rest position
        robot.moveArm(ARM_REST_POSITION);
        robot.waitForSystem(500, systems.ARM);

        // Move backward and close claws
        robot.moveBackward(150);
        robot.moveClaws(true, true, CLAWS_CLOSED);
        robot.waitForSystem(20, systems.WHEELS, systems.CLAWS);

        // Crabwalk left
        robot.moveLeft(1600);
        robot.waitForSystem(20, systems.WHEELS);

        // Move forward
        robot.moveForward(700);
        robot.waitForSystem(0, systems.WHEELS);
    }
}