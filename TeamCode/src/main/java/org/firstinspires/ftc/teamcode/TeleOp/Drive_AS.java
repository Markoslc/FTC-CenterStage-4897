package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.Robot.RobotParameters.*;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.Robot.Controller;
import org.firstinspires.ftc.teamcode.Robot.Robot;

@TeleOp(name = "Drive AS")
public class Drive_AS extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot      robot              = new Robot(DrivePeriod.DRIVER, false, 1, this); // Change imu reset to false
        Controller driverController   = new Controller(gamepad1);
        Controller operatorController = new Controller(gamepad2);

        waitForStart();

        robot.start();

        while (opModeIsActive()) {

            //
            // Driver Controller
            //
            driverController.updateInputs();

            double forwardPower  = driverController.leftStick.getY();
            double sidePower     = driverController.leftStick.getX();
            double rotationPower = driverController.rightStick.getX();
            robot.drive(forwardPower, sidePower, rotationPower);

            if (driverController.leftBumper.pressed())
                robot.moveClaws(true, false, ClawPositions.CLAWS_OPEN);
            else robot.moveClaws(true, false, ClawPositions.CLAWS_CLOSED);
            if (driverController.rightBumper.pressed())
                robot.moveClaws(false, true, ClawPositions.CLAWS_OPEN);
            else robot.moveClaws(false, true, ClawPositions.CLAWS_CLOSED);

            if (driverController.dpadUp.singlePress())
                robot.nextArmPos();
            if (driverController.dpadDown.singlePress())
                robot.prevArmPos();

            if (driverController.a.singlePress()) robot.moveLift();
            if (driverController.a.onRelease()) robot.switchLiftPosition();

            //
            // Operator controller
            //
            operatorController.updateInputs();

            if (operatorController.a.singlePress())
                robot.switchDriveMode();

            if (operatorController.b.singlePress())
                robot.setWheelPower(0.6);
            if (operatorController.b.onRelease())
                robot.setWheelPower(1);

            if (operatorController.leftBumper.pressed() && operatorController.rightBumper.pressed())
                robot.launchPlane();

            robot.update();
        }
    }
}
