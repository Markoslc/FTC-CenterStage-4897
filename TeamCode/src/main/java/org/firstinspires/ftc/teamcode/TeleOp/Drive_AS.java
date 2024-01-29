package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.RobotParameters.*;
import static org.firstinspires.ftc.teamcode.Robot.*;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name = "Drive AS")
public class Drive_AS extends LinearOpMode {
    @Override
    public void runOpMode(){
        Robot robot =  new Robot(false, 1, this);

        waitForStart();
        while(opModeIsActive()){

            double forwardPower = -gamepad1.left_stick_y;
            double sidePower = gamepad1.left_stick_x;
            double rotationPower = gamepad1.right_stick_x;

            robot.drive(forwardPower, sidePower, rotationPower);

            if(gamepad1.left_bumper) robot.moveClaws(true, false, ClawPositions.CLAWS_OPEN);
            else robot.moveClaws(true, false, ClawPositions.CLAWS_CLOSED);

            if(gamepad1.right_bumper) robot.moveClaws(false, true, ClawPositions.CLAWS_OPEN);
            else robot.moveClaws(true, false, ClawPositions.CLAWS_CLOSED);

            if(gamepad1.dpad_up) robot.nextArmPos();
            if(gamepad1.dpad_down) robot.prevArmPos();

            if(gamepad1.a){
                if(currLiftDirection == LiftDirections.REST) {
                    currLiftDirection = nextLiftDirection;
                    nextLiftDirection = nextLiftDirection == LiftDirections.UP ? LiftDirections.DOWN : nextLiftDirection;
                }
            } else currLiftDirection = LiftDirections.REST;
            robot.moveLift(currLiftDirection);

            robot.update();
        }
    }
}
