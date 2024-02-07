package org.firstinspires.ftc.teamcode.Autonomous;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import static org.firstinspires.ftc.teamcode.Robot.RobotParameters.*;


import java.util.List;


@Autonomous(name = "Auto Guided Blue Short AS")
public class AutoGuidedDriveBlueShort extends LinearOpMode {
    public               Robot               robot;
    public               DriveModes.PixelPos pixelPos   = DriveModes.PixelPos.UNKNOWN;
    private static final boolean             USE_WEBCAM = true;  // true for webcam, false for phone camera

    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        robot = new Robot(DrivePeriod.AUTONOMOUS, true, 1, this);

        initTfod();

        waitForStart();

        pixelPos = searchPixel();

        switch (pixelPos) {
            case LEFT:
                useLeftMode();
                break;
            case RIGHT:
                useRightMode();
                break;
            case CENTER:
            case UNKNOWN:
                useCenterMode();
                break;
        }
    }

    private void useRightMode() {
        telemetry.addLine("Mode: RightMode");
        telemetry.update();

        robot.turnAngleRight(-25);

        robot.moveArm(ARM_LOAD_POS - 50, true);

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
    }

    private void useLeftMode() {
        telemetry.addLine("Mode: LeftMode");
        telemetry.update();

        robot.moveArm(ARM_LOAD_POS - 25, true);

        robot.turnAngleLeft(25);

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
    }

    private void useCenterMode() {
        telemetry.addLine("Mode: CenterMode");
        telemetry.update();

        robot.moveForward(375, true);

        robot.moveArm(ARM_LOAD_POS - 25, true);

        robot.moveClaws(false, true, ClawPositions.CLAWS_FALL, true);

        robot.moveArm(ARM_REST_POS);
        robot.moveClaws(false, true, ClawPositions.CLAWS_CLOSED);
        robot.waitForSystem(20, Systems.CLAWS, Systems.WHEELS, Systems.ARM);

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
    }

    public DriveModes.PixelPos searchPixel() {
        robot.moveForward(325);
        robot.moveArm(ARM_LOAD_POS);
        robot.waitForSystem(1200, Systems.WHEELS, Systems.ARM);

        if (pixelInView()) {
            return DriveModes.PixelPos.CENTER;
        }

        robot.moveArm(ARM_SCORE_POS, true);

        robot.turnAngleLeft(15);

        robot.moveArm(ARM_LOAD_POS, true);

        robot.moveBackward(100);
        robot.waitForSystem(250, Systems.WHEELS);

        if (pixelInView()) {
            return DriveModes.PixelPos.LEFT;

        } else {
            robot.moveForward(100, true);

            robot.moveArm(ARM_SCORE_POS, true);

            return DriveModes.PixelPos.RIGHT;
        }

    }

    public DriveModes.PixelPos getPixelPos() {
        if (visionPortal == null) {
            initTfod();
        }
        //visionPortal.stopStreaming();
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        if (!currentRecognitions.isEmpty()) {
            Recognition highestConfidenceRecognition = null;
            double      maxConfidence                = 0.0;

            for (Recognition recognition : currentRecognitions) {
                double confidence = recognition.getConfidence();

                if (confidence > maxConfidence) {
                    maxConfidence = confidence;
                    highestConfidenceRecognition = recognition;
                }
            }

            AngleUnit unit = AngleUnit.DEGREES;
            assert highestConfidenceRecognition != null;
            double degrees = highestConfidenceRecognition.estimateAngleToObject(unit); //TODO: this is interesting for the pickup assistant
            double x       = (highestConfidenceRecognition.getLeft() + highestConfidenceRecognition.getRight()) / 2;
            double y       = (highestConfidenceRecognition.getTop() + highestConfidenceRecognition.getBottom()) / 2;
            telemetry.addData("", " ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", highestConfidenceRecognition.getLabel(), highestConfidenceRecognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", highestConfidenceRecognition.getWidth(), highestConfidenceRecognition.getHeight());
            if (x < 50) {
                return DriveModes.PixelPos.LEFT;
            } else if (x > 180) {
                return DriveModes.PixelPos.CENTER;
            } else {
                return DriveModes.PixelPos.RIGHT;
            }
        } else {
            return DriveModes.PixelPos.UNKNOWN;
        }
    }

    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder().build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .addProcessor(tfod)
                .setCameraResolution(new Size(1280, 720));


        visionPortal = builder.build();


    } //initTfod

    public boolean pixelInView() {
        int     i = 0;
        boolean result;
        while (i < 500) {
            i++;
            List<Recognition> currentRecognitions = tfod.getRecognitions();

            result = currentRecognitions.size() > 0;
            if (result) {
                return true;
            }
        }
        List<Recognition> currentRecognitions = tfod.getRecognitions();

        return currentRecognitions.size() > 0;
    }
}