package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Robot.RobotParameters.DrivePeriod;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Controller;
import org.firstinspires.ftc.teamcode.Robot.Robot;

@TeleOp(name = "Test: Dashboard", group = "Test")
public class DashboardTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot      robot              = new Robot(DrivePeriod.TEST, false, 1, this); // Change imu reset to false
        Controller driverController   = new Controller(gamepad1);
        Controller operatorController = new Controller(gamepad2);

        waitForStart();
        while (opModeIsActive()) {

            //
            // Driver Controller
            //
            driverController.updateInputs();

            double forwardPower;
            double sidePower;

            if (driverController.y.pressed()) forwardPower = 1;
            else if (driverController.a.pressed()) forwardPower = 1;
            else forwardPower = 0;

            if (driverController.b.pressed()) sidePower = 1;
            else if (driverController.x.pressed()) sidePower = -1;
            else sidePower = 0;

            robot.drive(forwardPower, sidePower, 0);

            //
            // Operator controller
            //
            operatorController.updateInputs();

            robot.update();
        }
    }
}
