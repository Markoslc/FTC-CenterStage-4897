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
        Robot robot = new Robot(DrivePeriod.AUTONOMOUS, true, 0.75, this);

        robot.moveClaws(true, true, ClawPositions.CLAWS_CLOSED);

        waitForStart();

        // Move forward
        robot.moveForward(1360);
        robot.waitForSystem(20, Systems.WHEELS);

        // Move backward
        robot.moveBackward(360);
        robot.waitForSystem(20, Systems.WHEELS);

        // Turn right
        robot.turnRight(900);
        robot.waitForSystem(20, Systems.WHEELS);

        // Move forward
        robot.moveForward(1550);
        robot.waitForSystem(20, Systems.WHEELS);

        // Move arm to placing position
        robot.moveArm(ARM_SCORE_POS);
        robot.waitForSystem(500, Systems.ARM);

        // Open claws
        robot.moveClaws(true, true, ClawPositions.CLAWS_FALL);
        robot.waitForSystem(500, Systems.CLAWS);

        // Move arm to rest position
        robot.moveArm(ARM_REST_POS);
        robot.waitForSystem(500, Systems.ARM);

        // Move backward and close claws
        robot.moveBackward(150);
        robot.moveClaws(true, true, ClawPositions.CLAWS_CLOSED);
        robot.waitForSystem(20, Systems.WHEELS, Systems.CLAWS);

        // Crabwalk left
        robot.moveLeft(1600);
        robot.waitForSystem(20, Systems.WHEELS);

        // Move forward
        robot.moveForward(700);
        robot.waitForSystem(0, Systems.WHEELS);
    }
}