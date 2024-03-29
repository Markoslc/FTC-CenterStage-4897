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

@Autonomous(name = "Blue Long Team Prop", group = "Team Prop")
public class BlueLongTeamProp extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot               robot               = new Robot(DrivePeriod.AUTONOMOUS, true, 0.75, this);
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

                robot.moveForward(775, true);

                robot.moveClaws(false, true, ClawPositions.CLAWS_FALL, true);

                robot.moveBackward(500, true);
                robot.moveArm(ARM_REST_POS);
                robot.moveClaws(false, true, ClawPositions.CLAWS_CLOSED);
                robot.waitForSystem(50, Systems.CLAWS, Systems.WHEELS, Systems.ARM);

                robot.turnAngleLeft(90);

                robot.moveLeft(200, true);

                robot.moveForward(3000, true);

                robot.moveRight(1400, true);

                robot.moveForward(925, true);

                robot.moveArm(ARM_SCORE_POS, true);

                robot.moveClaws(true, false, ClawPositions.CLAWS_FALL, true);

                robot.moveBackward(300, true);

                robot.moveRight(1400, true);

                robot.moveArm(ARM_REST_POS);
                robot.moveClaws(true, false, ClawPositions.CLAWS_CLOSED);
                robot.moveForward(700);
                robot.waitForSystem(20, Systems.CLAWS, Systems.WHEELS, Systems.ARM);
                break;
            case LEFT:
                telemetry.addLine("Mode: LeftMode");
                telemetry.update();

                robot.moveForward(650, true);

                robot.turnAngleLeft(50);

                robot.moveBackward(500, true);

                robot.moveArm(ARM_LOAD_POS, true);

                robot.moveForward(350, true);

                robot.moveClaws(false, true, ClawPositions.CLAWS_FALL, true);

                robot.moveArm(ARM_REST_POS);
                robot.moveClaws(false, true, ClawPositions.CLAWS_CLOSED);
                robot.moveBackward(200);
                robot.waitForSystem(50, Systems.CLAWS, Systems.WHEELS, Systems.ARM);

                robot.turnAngleLeft(90);

                robot.moveLeft(400, true);

                robot.moveForward(3000, true);

                robot.moveRight(1400, true);

                robot.moveForward(950, true);

                robot.moveArm(ARM_SCORE_POS, true);

                robot.moveClaws(true, false, ClawPositions.CLAWS_FALL, true);

                robot.moveBackward(300, true);

                robot.moveRight(1200, true);

                robot.moveArm(ARM_REST_POS);
                robot.moveClaws(true, false, ClawPositions.CLAWS_CLOSED);
                robot.moveForward(700);
                robot.waitForSystem(20, Systems.CLAWS, Systems.WHEELS, Systems.ARM);
                break;
            case RIGHT:
                telemetry.addLine("Mode: RightMode");
                telemetry.update();

                robot.moveForward(350, true);

                robot.turnAngleRight(-30);

                robot.moveBackward(200, true);

                robot.moveArm(ARM_LOAD_POS, true);

                robot.moveForward(350, true);

                robot.moveClaws(false, true, ClawPositions.CLAWS_FALL, true);

                robot.moveArm(ARM_REST_POS);
                robot.moveClaws(false, true, ClawPositions.CLAWS_CLOSED);
                robot.waitForSystem(50, Systems.CLAWS, Systems.ARM);

                robot.turnAngleLeft(90);

                robot.moveLeft(400, true);

                robot.moveForward(3000, true);

                robot.moveRight(1400, true);

                robot.moveForward(950, true);

                robot.moveArm(ARM_SCORE_POS, true);

                robot.moveClaws(true, false, ClawPositions.CLAWS_FALL, true);

                robot.moveBackward(300, true);

                robot.moveRight(1200, true);

                robot.moveArm(ARM_REST_POS);
                robot.moveClaws(true, false, ClawPositions.CLAWS_CLOSED);
                robot.moveForward(700);
                robot.waitForSystem(20, Systems.CLAWS, Systems.WHEELS, Systems.ARM);
                break;
        }

    }
}
