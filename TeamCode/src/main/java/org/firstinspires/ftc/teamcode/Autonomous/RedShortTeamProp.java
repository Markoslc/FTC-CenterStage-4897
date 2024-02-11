package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.OpenCV.TeamPropRecognition;
import org.firstinspires.ftc.teamcode.Robot.Robot;

import static org.firstinspires.ftc.teamcode.Robot.RobotParameters.*;

@Autonomous(name = "Red Short Team Prop", group = "Team Prop")
public class RedShortTeamProp extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot               robot               = new Robot(DrivePeriod.AUTONOMOUS, true, 1, this);
        TeamPropRecognition teamPropRecognition = new TeamPropRecognition(this, Alliance.RED_ALLIANCE);

        waitForStart();

        TeamPropPosition teamPropPosition = teamPropRecognition.getTeamPropPosition();

        teamPropRecognition.stopCameraStream();

        switch (teamPropPosition) {
            case CENTER:
                telemetry.addLine("Mode: CenterMode");
                telemetry.update();

                robot.moveArm(ARM_LOAD_POS, true);

                robot.moveForward(800, true);

                robot.moveClaws(false, true, ClawPositions.CLAWS_FALL, true);

                robot.moveBackward(200);
                robot.moveArm(ARM_REST_POS);
                robot.moveClaws(false, true, ClawPositions.CLAWS_CLOSED);
                robot.waitForSystem(20, Systems.CLAWS, Systems.WHEELS, Systems.ARM);

                robot.moveRight(800, true);

                robot.turnAngleRight(-90);

                robot.moveLeft(400, true);

                robot.moveForward(1000, true);

                robot.moveArm(ARM_SCORE_POS, true);

                robot.moveClaws(true, false, ClawPositions.CLAWS_FALL, true);

                robot.moveBackward(200, true);

                robot.moveLeft(1200, true);

                robot.moveArm(ARM_REST_POS);
                robot.moveClaws(true, false, ClawPositions.CLAWS_CLOSED);
                robot.moveForward(700);
                robot.waitForSystem(20, Systems.CLAWS, Systems.WHEELS, Systems.ARM);
                break;
            case LEFT:
                telemetry.addLine("Mode: LeftMode");
                telemetry.update();

                robot.moveForward(350, true);

                robot.turnAngleLeft(25);

                robot.moveArm(ARM_LOAD_POS, true);

                robot.moveForward(350, true);

                robot.moveClaws(false, true, ClawPositions.CLAWS_FALL, true);

                robot.moveBackward(200);
                robot.moveArm(ARM_REST_POS);
                robot.moveClaws(false, true, ClawPositions.CLAWS_CLOSED);
                robot.waitForSystem(50, Systems.CLAWS, Systems.WHEELS, Systems.ARM);

                robot.turnAngleRight(-90);

                robot.moveForward(1250, true);

                robot.moveLeft(1100, true);

                robot.moveForward(375, true);

                robot.moveArm(ARM_SCORE_POS, true);

                robot.moveClaws(true, false, ClawPositions.CLAWS_FALL, true);

                robot.moveBackward(200, true);

                robot.moveLeft(1200, true);

                robot.moveForward(500);
                robot.moveArm(ARM_REST_POS);
                robot.moveClaws(true, true, ClawPositions.CLAWS_CLOSED);
                robot.waitForSystem(20, Systems.CLAWS, Systems.WHEELS, Systems.ARM);
                break;
            case RIGHT:
                telemetry.addLine("Mode: RightMode");
                telemetry.update();

                robot.moveForward(325, true);

                robot.turnAngleRight(-25);

                robot.moveArm(ARM_LOAD_POS, true);

                robot.moveForward(300, true);

                robot.moveClaws(false, true, ClawPositions.CLAWS_FALL, true);

                robot.moveArm(ARM_REST_POS);
                robot.moveClaws(false, true, ClawPositions.CLAWS_CLOSED);
                robot.waitForSystem(50, Systems.CLAWS, Systems.ARM);

                robot.turnAngleRight(-90);

                robot.moveForward(1250, true);

                robot.moveLeft(1100, true);

                robot.moveForward(325, true);

                robot.moveArm(ARM_SCORE_POS, true);

                robot.moveClaws(true, false, ClawPositions.CLAWS_FALL, true);

                robot.moveBackward(200, true);

                robot.moveLeft(1200, true);

                robot.moveForward(500);
                robot.moveArm(ARM_REST_POS);
                robot.moveClaws(true, true, ClawPositions.CLAWS_CLOSED);
                robot.waitForSystem(20, Systems.CLAWS, Systems.WHEELS, Systems.ARM);
                break;
        }

    }
}
