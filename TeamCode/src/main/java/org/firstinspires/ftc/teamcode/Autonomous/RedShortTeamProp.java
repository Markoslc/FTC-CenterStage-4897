package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Robot.RobotParameters.ARM_LOAD_POS;
import static org.firstinspires.ftc.teamcode.Robot.RobotParameters.ARM_REST_POS;
import static org.firstinspires.ftc.teamcode.Robot.RobotParameters.ARM_SCORE_POS;
import static org.firstinspires.ftc.teamcode.Robot.RobotParameters.ClawPositions;
import static org.firstinspires.ftc.teamcode.Robot.RobotParameters.DrivePeriod;
import static org.firstinspires.ftc.teamcode.Robot.RobotParameters.Systems;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.OpenCV.TeamPropRecognition;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.RobotParameters;

@Autonomous(name = "Red Short Team Prop")
public class RedShortTeamProp extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = new Robot(DrivePeriod.DRIVER, true, 1, this);
        TeamPropRecognition teamPropRecognition = new TeamPropRecognition(this, RobotParameters.Alliance.RED_ALLIANCE);

        waitForStart();

        switch (teamPropRecognition.getTeamPropPosition()){
            case CENTER:
                robot.moveArm(ARM_LOAD_POS, true);

                robot.moveForward(800, true);

                robot.moveClaws(false, true, ClawPositions.CLAWS_FALL, true);

                robot.moveBackward(200, true);

                robot.turnAngleRight(-90);
                robot.moveArm(ARM_REST_POS);
                robot.moveClaws(false, true, ClawPositions.CLAWS_CLOSED);
                robot.waitForSystem(50, Systems.CLAWS, Systems.WHEELS, Systems.ARM);

                robot.moveForward(1200, true);

                robot.moveLeft(500, true);

                robot.moveForward(200);
                robot.moveArm(ARM_SCORE_POS);
                robot.waitForSystem(50, Systems.WHEELS, Systems.ARM);

                robot.moveClaws(true, false, ClawPositions.CLAWS_FALL, true);

                robot.moveBackward(200);

                robot.moveLeft(750, true);

                robot.moveForward(500);
                robot.moveArm(ARM_REST_POS);
                robot.moveClaws(true, false, ClawPositions.CLAWS_CLOSED);
                robot.waitForSystem(20, Systems.CLAWS, Systems.WHEELS, Systems.ARM);
                break;
            case LEFT:
                robot.moveForward(500);

                robot.turnAngleLeft(22.5);

                robot.moveArm(ARM_LOAD_POS, true);

                robot.moveForward(500, true);

                robot.moveClaws(false, true, ClawPositions.CLAWS_FALL, true);

                robot.moveBackward(500, true);

                robot.turnAngleLeft(90);
                robot.moveArm(ARM_REST_POS);
                robot.moveClaws(false, true, ClawPositions.CLAWS_CLOSED);
                robot.waitForSystem(50, Systems.CLAWS, Systems.WHEELS, Systems.ARM);

                robot.moveForward(1200, true);

                robot.moveLeft(500, true);

                robot.moveForward(200);
                robot.moveArm(ARM_SCORE_POS);
                robot.waitForSystem(50, Systems.WHEELS, Systems.ARM);

                robot.moveClaws(true, false, ClawPositions.CLAWS_FALL, true);

                robot.moveBackward(200);

                robot.moveLeft(750, true);

                robot.moveForward(500);
                robot.moveArm(ARM_REST_POS);
                robot.moveClaws(true, false, ClawPositions.CLAWS_CLOSED);
                robot.waitForSystem(20, Systems.CLAWS, Systems.WHEELS, Systems.ARM);
                break;
            case RIGHT:
                robot.moveForward(500);

                robot.turnAngleLeft(-22.5);

                robot.moveArm(ARM_LOAD_POS, true);

                robot.moveForward(500, true);

                robot.moveClaws(false, true, ClawPositions.CLAWS_FALL, true);

                robot.moveBackward(500, true);

                robot.turnAngleLeft(90);
                robot.moveArm(ARM_REST_POS);
                robot.moveClaws(false, true, ClawPositions.CLAWS_CLOSED);
                robot.waitForSystem(50, Systems.CLAWS, Systems.WHEELS, Systems.ARM);

                robot.moveForward(1200, true);

                robot.moveLeft(500, true);

                robot.moveForward(200);
                robot.moveArm(ARM_SCORE_POS);
                robot.waitForSystem(50, Systems.WHEELS, Systems.ARM);

                robot.moveClaws(true, false, ClawPositions.CLAWS_FALL, true);

                robot.moveBackward(200);

                robot.moveLeft(750, true);

                robot.moveForward(500);
                robot.moveArm(ARM_REST_POS);
                robot.moveClaws(true, false, ClawPositions.CLAWS_CLOSED);
                robot.waitForSystem(20, Systems.CLAWS, Systems.WHEELS, Systems.ARM);
                break;
        }

    }
}
