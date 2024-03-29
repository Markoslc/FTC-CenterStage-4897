package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Robot.RobotParameters.ARM_LOAD_POS;
import static org.firstinspires.ftc.teamcode.Robot.RobotParameters.ARM_REST_POS;
import static org.firstinspires.ftc.teamcode.Robot.RobotParameters.ARM_SCORE_POS;
import static org.firstinspires.ftc.teamcode.Robot.RobotParameters.Alliance;
import static org.firstinspires.ftc.teamcode.Robot.RobotParameters.ClawPositions;
import static org.firstinspires.ftc.teamcode.Robot.RobotParameters.DrivePeriod;
import static org.firstinspires.ftc.teamcode.Robot.RobotParameters.Systems;
import static org.firstinspires.ftc.teamcode.Robot.RobotParameters.TeamPropPosition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.OpenCV.TeamPropRecognition;
import org.firstinspires.ftc.teamcode.Robot.Robot;

@Autonomous(name = "Blue Short Team Prop", group = "Team Prop")
public class BlueShortTeamProp extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot               robot               = new Robot(DrivePeriod.AUTONOMOUS, true, 0.6, this);
        TeamPropRecognition teamPropRecognition = new TeamPropRecognition(this, Alliance.BLUE_ALLIANCE);

        while(!isStarted()){
            telemetry.addData("Team prop position:", teamPropRecognition.getTeamPropPosition());
            telemetry.update();
        }

        robot.start();

        TeamPropPosition teamPropPosition = teamPropRecognition.getTeamPropPosition();

        teamPropRecognition.stopCameraStream();

        switch (teamPropPosition) {
            case CENTER:
                telemetry.addLine("Mode: CenterMode");
                telemetry.update();
                robot.moveArm(ARM_LOAD_POS, true);

                robot.moveForward(700, true);

                robot.moveClaws(false, true, ClawPositions.CLAWS_FALL, true);

                robot.moveBackward(200, true);
                robot.moveArm(ARM_REST_POS);
                robot.moveClaws(false, true, ClawPositions.CLAWS_CLOSED);
                robot.waitForSystem(50, Systems.CLAWS, Systems.WHEELS, Systems.ARM);

                robot.moveLeft(800, true);

                robot.turnAngleLeft(90);

                robot.moveRight(1100, true);

                robot.moveForward(1050, true);

                robot.moveArm(ARM_SCORE_POS, true);

                robot.moveClaws(true, false, ClawPositions.CLAWS_FALL, true);

                robot.moveBackward(300, true);

                robot.moveRight(1050, true);

                robot.moveForward(600);
                robot.moveArm(ARM_REST_POS);
                robot.moveClaws(true, false, ClawPositions.CLAWS_CLOSED);
                robot.waitForSystem(20, Systems.CLAWS, Systems.WHEELS, Systems.ARM);
                break;
            case LEFT:
                telemetry.addLine("Mode: LeftMode");
                telemetry.update();

                robot.moveForward(350, true);

                robot.turnAngleLeft(25);

                robot.moveBackward(250, true);

                robot.moveArm(ARM_LOAD_POS, true);

                robot.moveForward(350, true);

                robot.moveClaws(false, true, ClawPositions.CLAWS_FALL, true);

                robot.moveArm(ARM_REST_POS);
                robot.moveClaws(false, true, ClawPositions.CLAWS_CLOSED);
                robot.waitForSystem(50, Systems.CLAWS, Systems.ARM);

                robot.turnAngleLeft(90);

                robot.moveForward(1300, true);

                robot.moveRight(1175, true);

                robot.moveForward(275, true);

                robot.moveArm(ARM_SCORE_POS, true);

                robot.moveClaws(true, false, ClawPositions.CLAWS_FALL, true);

                robot.moveBackward(100, true);

                robot.moveRight(1150, true);

                robot.moveForward(500);
                robot.moveArm(ARM_REST_POS);
                robot.moveClaws(true, false, ClawPositions.CLAWS_CLOSED);
                robot.waitForSystem(20, Systems.CLAWS, Systems.WHEELS, Systems.ARM);
                break;
            case RIGHT:
                telemetry.addLine("Mode: RightMode");
                telemetry.update();

                robot.moveForward(650, true);

                robot.turnAngleRight(-45);

                robot.moveBackward(500, true);

                robot.moveArm(ARM_LOAD_POS, true);

                robot.moveForward(300, true);

                robot.moveClaws(false, true, ClawPositions.CLAWS_FALL, true);

                robot.moveArm(ARM_REST_POS);
                robot.moveClaws(false, true, ClawPositions.CLAWS_CLOSED);
                robot.moveBackward(200);
                robot.waitForSystem(50, Systems.CLAWS, Systems.WHEELS, Systems.ARM);

                robot.turnAngleLeft(90);

                robot.moveForward(1075, true);

                robot.moveRight(1000, true);

                robot.moveForward(375, true);

                robot.moveArm(ARM_SCORE_POS, true);

                robot.moveClaws(true, false, ClawPositions.CLAWS_FALL, true);

                robot.moveBackward(200, true);

                robot.moveRight(1375, true);

                robot.moveForward(400);
                robot.moveArm(ARM_REST_POS);
                robot.moveClaws(true, false, ClawPositions.CLAWS_CLOSED);
                robot.waitForSystem(20, Systems.CLAWS, Systems.WHEELS, Systems.ARM);
                break;
        }

    }
}
