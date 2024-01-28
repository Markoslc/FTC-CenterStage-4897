package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Orientation.AprilTagRecognition;
import org.firstinspires.ftc.teamcode.Orientation.Position2D;
import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name = "Test", group = "Test")
public class Test extends LinearOpMode {
    AprilTagRecognition tags;
    Position2D botPosition;
    @Override
    public void runOpMode(){
        Robot robot =  new Robot(false, 1, this);

        waitForStart();
        while(opModeIsActive()){
            tags = new AprilTagRecognition(this);
            botPosition = tags.getRobotPosition();
            telemetry.addLine(botPosition.toString());

            robot.update();
        }
    }
}