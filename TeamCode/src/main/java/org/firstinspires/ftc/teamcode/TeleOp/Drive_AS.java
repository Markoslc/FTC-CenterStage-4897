package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotParameters;

@TeleOp(name = "Drive AS")
public class Drive_AS extends LinearOpMode {
    @Override
    public void runOpMode(){
        Robot robot =  new Robot(false, 1);

        waitForStart();
        while(opModeIsActive()){
            double forwardPower = -gamepad1.left_stick_y;
            double sidePower = gamepad1.left_stick_x;
            double rotationPower = gamepad1.right_stick_x / 3;

            robot.drive(forwardPower, sidePower, rotationPower);

            if(gamepad1.left_bumper) robot.moveClaws(true, false, RobotParameters.ClawPositions.CLAWS_OPEN);
            else robot.moveClaws(true, false, RobotParameters.ClawPositions.CLAWS_CLOSED);

            if(gamepad1.right_bumper) robot.moveClaws(false, true, RobotParameters.ClawPositions.CLAWS_OPEN);
            else robot.moveClaws(true, false, RobotParameters.ClawPositions.CLAWS_CLOSED);


        }
    }
}
