package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.OpenCV.TeamPropRecognition;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import static org.firstinspires.ftc.teamcode.Robot.RobotParameters.*;

@Autonomous(name = "Blue Short Team Prop")
public class BlueShortTeamProp extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = new Robot(DrivePeriod.AUTONOMOUS, true, 1, this);
        TeamPropRecognition teamPropRecognition = new TeamPropRecognition(this, Alliance.BLUE_ALLIANCE);

        waitForStart();

        switch (teamPropRecognition.getTeamPropPosition()){
            case CENTER:
                telemetry.addLine("Mode: CenterMode");
                telemetry.update();
                robot.moveArm(ARM_LOAD_POS, true);

                robot.moveForward(800, true);

                robot.moveClaws(false, true, ClawPositions.CLAWS_FALL, true);

                robot.moveBackward(200, true);
                robot.moveArm(ARM_REST_POS);
                robot.moveClaws(false, true, ClawPositions.CLAWS_CLOSED);
                robot.waitForSystem(50, Systems.CLAWS, Systems.WHEELS, Systems.ARM);

                robot.moveLeft(800, true);

                robot.turnAngleLeft(90);

                robot.moveRight(1100, true);

                robot.moveForward(850, true);

                robot.moveArm(ARM_SCORE_POS, true);

                robot.moveClaws(true, false, ClawPositions.CLAWS_FALL, true);

                robot.moveBackward(300, true);

                robot.moveRight(1000, true);

                robot.moveArm(ARM_REST_POS);
                robot.moveClaws(true, false, ClawPositions.CLAWS_CLOSED);
                robot.moveForward(700);
                robot.waitForSystem(20, Systems.CLAWS, Systems.WHEELS, Systems.ARM);
                break;
            case LEFT:
                robot.moveForward(325, true);

                robot.turnAngleLeft(25);

                robot.moveArm(ARM_LOAD_POS, true);

                robot.moveForward(200, true);

                robot.moveClaws(false, true, ClawPositions.CLAWS_FALL, true);

                robot.moveArm(ARM_REST_POS);
                robot.moveClaws(false, true, ClawPositions.CLAWS_CLOSED);
                robot.waitForSystem(50, Systems.CLAWS, Systems.ARM);

                robot.turnAngleLeft(90);

                robot.moveForward(1600, true);

                robot.moveRight(1500, true);

                robot.moveArm(ARM_SCORE_POS, true);

                robot.moveClaws(true, false, ClawPositions.CLAWS_FALL, true);

                robot.moveBackward(100, true);

                robot.moveRight(1000, true);

                robot.moveForward(500, true);
                robot.moveArm(ARM_REST_POS);
                robot.moveClaws(true, false, ClawPositions.CLAWS_CLOSED);
                robot.waitForSystem(20, Systems.CLAWS, Systems.WHEELS, Systems.ARM);
                break;
            case RIGHT:
                telemetry.addLine("Mode: RightMode");
                telemetry.update();
                robot.moveForward(325);

                robot.turnAngleLeft(25);

                robot.moveArm(ARM_LOAD_POS, true);

                robot.moveForward(250, true);

                robot.moveClaws(false, true, ClawPositions.CLAWS_FALL, true);

                robot.moveArm(ARM_REST_POS);
                robot.moveClaws(false, true, ClawPositions.CLAWS_CLOSED);
                robot.moveBackward(200);
                robot.waitForSystem(50, Systems.CLAWS, Systems.WHEELS, Systems.ARM);

                robot.turnAngleLeft(90);

                robot.moveForward(1250, true);

                robot.moveRight(1500, true);

                robot.moveForward(375, true);

                robot.moveArm(ARM_SCORE_POS, true);

                robot.moveClaws(true, false, ClawPositions.CLAWS_FALL, true);

                robot.moveBackward(200, true);

                robot.moveRight(1000, true);

                robot.moveForward(500, true);
                robot.moveArm(ARM_REST_POS);
                robot.moveClaws(true, false, ClawPositions.CLAWS_CLOSED);
                robot.waitForSystem(20, Systems.CLAWS, Systems.WHEELS, Systems.ARM);
                break;
        }

    }
}
