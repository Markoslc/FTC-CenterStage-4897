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

@Autonomous(name = "Red Long Team Prop", group = "Team Prop")
public class RedLongTeamProp extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot               robot               = new Robot(DrivePeriod.AUTONOMOUS, true, 0.6, this);
        TeamPropRecognition teamPropRecognition = new TeamPropRecognition(this, Alliance.RED_ALLIANCE);

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

                robot.moveBackward(450, true);
                robot.moveArm(ARM_REST_POS);
                robot.moveClaws(false, true, ClawPositions.CLAWS_CLOSED);
                robot.waitForSystem(50, Systems.CLAWS, Systems.WHEELS, Systems.ARM);

                robot.turnAngleRight(-90);

                //robot.moveRight(150, true);

                // from here only for round 15:
                //robot.moveForward(3700, true);


                robot.moveForward(3000, true);

                robot.moveLeft(1400, true);

                robot.moveForward(950, true);

                robot.moveArm(ARM_SCORE_POS, true);

                robot.moveClaws(true, false, ClawPositions.CLAWS_FALL, true);

                robot.moveBackward(300, true);

                robot.moveRight(1300, true);

                robot.moveArm(ARM_REST_POS);
                robot.moveClaws(true, false, ClawPositions.CLAWS_CLOSED);
                robot.moveForward(700);
                 /*
                robot.moveArm(ARM_LOAD_POS, true);
                robot.moveClaws(true, true, ClawPositions.CLAWS_OPEN);
                robot.moveArm(ARM_REST_POS);
                robot.moveClaws(true, true, ClawPositions.CLAWS_CLOSED);
                robot.moveForward(600, true);
                */
                robot.waitForSystem(20, Systems.CLAWS, Systems.WHEELS, Systems.ARM);
                break;
            case LEFT:
                telemetry.addLine("Mode: LeftMode");
                telemetry.update();

                robot.moveForward(350, true);

                robot.turnAngleLeft(25);

                robot.moveBackward(275, true);

                robot.moveArm(ARM_LOAD_POS, true);

                robot.moveForward(375, true);

                robot.moveClaws(false, true, ClawPositions.CLAWS_FALL, true);

                robot.moveBackward(200, true);
                robot.moveArm(ARM_REST_POS);
                robot.moveClaws(false, true, ClawPositions.CLAWS_CLOSED);
                robot.waitForSystem(50, Systems.CLAWS, Systems.WHEELS, Systems.ARM);

                robot.turnAngleRight(-90);

                robot.moveRight(200, true);
                // from here only for round 15:
                //robot.moveForward(3700, true);

                robot.moveForward(3000, true);

                robot.moveLeft(1400, true);

                robot.moveForward(950, true);

                robot.moveArm(ARM_SCORE_POS, true);

                robot.moveClaws(true, false, ClawPositions.CLAWS_FALL, true);

                robot.moveBackward(300, true);

                robot.moveRight(1200, true);

                robot.moveArm(ARM_REST_POS);
                robot.moveClaws(true, false, ClawPositions.CLAWS_CLOSED);
                robot.moveForward(700);
                 /*
                robot.moveArm(ARM_LOAD_POS, true);
                robot.moveClaws(true, true, ClawPositions.CLAWS_OPEN);
                robot.moveArm(ARM_REST_POS);
                robot.moveClaws(true, true, ClawPositions.CLAWS_CLOSED);
                robot.moveForward(600, true);
                  */
                robot.waitForSystem(20, Systems.CLAWS, Systems.WHEELS, Systems.ARM);
                break;
            case RIGHT:
                telemetry.addLine("Mode: RightMode");
                telemetry.update();

                robot.moveForward(650, true);

                robot.turnAngleRight(-50);

                robot.moveBackward(500, true);

                robot.moveArm(ARM_LOAD_POS, true);

                robot.moveForward(325, true);

                robot.moveClaws(false, true, ClawPositions.CLAWS_FALL, true);

                robot.moveBackward(200);
                robot.moveArm(ARM_REST_POS);
                robot.moveClaws(false, true, ClawPositions.CLAWS_CLOSED);
                robot.waitForSystem(50, Systems.CLAWS, Systems.WHEELS, Systems.ARM);

                robot.turnAngleRight(-90);

                robot.moveRight(450, true);
                // from here only for round 15:
                //robot.moveForward(3700, true);

                robot.moveForward(3000, true);

                robot.moveLeft(1400, true);

                robot.moveForward(950, true);

                robot.moveArm(ARM_SCORE_POS, true);

                robot.moveClaws(true, false, ClawPositions.CLAWS_FALL, true);

                robot.moveBackward(300, true);

                robot.moveRight(1200, true);

                robot.moveArm(ARM_REST_POS);
                robot.moveClaws(true, false, ClawPositions.CLAWS_CLOSED);
                robot.moveForward(700);

                 /*
                robot.moveArm(ARM_LOAD_POS, true);
                robot.moveClaws(true, true, ClawPositions.CLAWS_OPEN);
                robot.moveArm(ARM_REST_POS);
                robot.moveClaws(true, true, ClawPositions.CLAWS_CLOSED);
                robot.moveForward(600, true);
                  */
                robot.waitForSystem(20, Systems.CLAWS, Systems.WHEELS, Systems.ARM);
                break;
        }

    }
}
