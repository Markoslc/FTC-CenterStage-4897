package org.firstinspires.ftc.teamcode.Tests;
import org.firstinspires.ftc.teamcode.Orientation.AprilTagRecognition;
import org.firstinspires.ftc.teamcode.Orientation.Position2D;
import org.firstinspires.ftc.teamcode.Robot.Controller;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import static org.firstinspires.ftc.teamcode.Robot.RobotParameters.*;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "AprilTagTest", group = "Test")
public class AprilTagTest extends LinearOpMode{
    @Override
    public void runOpMode() {
        Robot robot = new Robot(DrivePeriod.DRIVER, true, 0.5, this, Systems.ROBOT, Systems.WHEELS, Systems.IMU);
        AprilTagRecognition tags             = new AprilTagRecognition(this);
        Controller  driverController = new Controller(gamepad1);
        Position2D botPosition = new Position2D(0, 0, 0);
        try {
            botPosition.update(tags.getRobotPosition());
            telemetry.addLine("Init Position: " + botPosition);
        } catch (Exception e) {
            telemetry.addLine("Error: " + e.getMessage());
        }
        telemetry.update();

        waitForStart();
        robot.start();
        while (opModeIsActive()) {
            driverController.updateInputs();
            double forwardPower  = driverController.leftStick.getY();
            double sidePower     = driverController.leftStick.getX();
            double rotationPower = driverController.rightStick.getX();
            robot.drive(forwardPower, sidePower, rotationPower);
            telemetry.addData("forward:", forwardPower);
            telemetry.addData("side:", sidePower);
            telemetry.addData("rotation:", rotationPower);
            telemetry.update();
            Position2D pos = tags.getRobotPosition();
            if (pos != null) {
                botPosition.update(pos);
                telemetry.addData("Position: ", botPosition.toString());
                robot.update(Systems.ROBOT, Systems.WHEELS, Systems.IMU);
            }

        }
    }
}
