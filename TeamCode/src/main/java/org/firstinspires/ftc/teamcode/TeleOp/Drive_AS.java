package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.RobotParameters.*;
import static org.firstinspires.ftc.teamcode.Robot.*;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name = "Drive AS")
public class Drive_AS extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot   robot              = new Robot(DrivePeriod.DRIVER, true, 1, this); // Change imu reset to false
        boolean dpadUpWasPressed   = false;
        boolean dpadUpPressed      = false;
        boolean dpadDownWasPressed = false;
        boolean dpadDownPressed    = false;

        waitForStart();
        while (opModeIsActive()) {

            double forwardPower  = -gamepad1.left_stick_y;
            double sidePower     = gamepad1.left_stick_x;
            double rotationPower = gamepad1.right_stick_x;
            robot.drive(forwardPower, sidePower, rotationPower);

            telemetry.addData("left_stick_y", forwardPower);

            if (gamepad1.left_bumper && gamepad1.right_bumper)
                robot.moveClaws(true, true, ClawPositions.CLAWS_OPEN);
            else {
                if (gamepad1.left_bumper) robot.moveClaws(true, false, ClawPositions.CLAWS_OPEN);
                else robot.moveClaws(true, false, ClawPositions.CLAWS_CLOSED);
                // TODO: Set the right values for the position

                if (gamepad1.right_bumper) robot.moveClaws(false, true, ClawPositions.CLAWS_OPEN);
                else robot.moveClaws(false, true, ClawPositions.CLAWS_CLOSED);
                // TODO: Set the right values for the position
            }

            dpadUpWasPressed = dpadUpPressed;
            dpadUpPressed = false;
            if (gamepad1.dpad_up) {
                dpadUpPressed = true;
                if (!dpadUpWasPressed) robot.nextArmPos();
            }

            dpadDownWasPressed = dpadDownPressed;
            dpadDownPressed = false;
            if (gamepad1.dpad_down) {
                dpadDownPressed = true;
                if (!dpadDownWasPressed) robot.prevArmPos();
            }

            if (gamepad1.a) {
                if (currLiftDirection == LiftDirections.REST) {
                    currLiftDirection = nextLiftDirection;
                    nextLiftDirection = nextLiftDirection == LiftDirections.UP ? LiftDirections.DOWN : LiftDirections.UP;
                }
            } else currLiftDirection = LiftDirections.REST;
            robot.moveLift(currLiftDirection);

            robot.update();
        }
    }
}
