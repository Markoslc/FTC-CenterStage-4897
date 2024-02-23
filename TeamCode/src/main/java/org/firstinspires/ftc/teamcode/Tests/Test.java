package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Orientation.AprilTagRecognition;
import org.firstinspires.ftc.teamcode.Orientation.Position2D;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.RobotParameters;

@TeleOp(name = "Test", group = "Test")
public class Test extends LinearOpMode {
    AprilTagRecognition tags;
    Position2D          botPosition;

    @Override
    public void runOpMode() {
        Robot robot = new Robot(RobotParameters.DrivePeriod.DRIVER, true, 0.75, this);
        tags = new AprilTagRecognition(this);

        waitForStart();
        while (opModeIsActive()) {
            botPosition = tags.getRobotPosition();
            if (botPosition != null) telemetry.addLine(botPosition.toString());
            robot.update();
        }
    }
}