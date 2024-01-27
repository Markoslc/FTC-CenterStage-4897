package org.firstinspires.ftc.teamcode.Autonomous;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import static org.firstinspires.ftc.teamcode.RobotParameters.*;


import java.util.List;


@Autonomous(name = "Auto Guided Blue Short AS")
public class AutoGuidedDriveBlueShort extends LinearOpMode {
    public Robot robot;
    public DriveModes.PixelPos pixelPos = DriveModes.PixelPos.UNKNOWN;
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "MyModelStoredAsAsset.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/myCustomModel.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            "Pixel",
    };
    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        Robot robot = new Robot(true, 1, this);/*
        initClaws();
        initWheels();
        initArm();*/
        initTfod();

        waitForStart();
        
        robot.moveArm(ARM_LOAD_POS);
        robot.waitForSystem(500, Systems.ARM);

        pixelPos = getPixelPos();
        telemetry.addData("Pixel position = ", pixelPos.toString());
        telemetry.update();

        robot.moveArm(ARM_SCORE_POS, true);

        if (pixelPos == DriveModes.PixelPos.UNKNOWN){
            pixelPos = searchPixel();
        } else{
            robot.moveForward(200, true);
        }
        switch (pixelPos){
            case RIGHT:
                useRightMode();
                break;
            case LEFT:
                useLeftMode();
                break;
            case CENTER:
            case UNKNOWN:
                useCenterMode();
                break;
        }
    }

    private void useRightMode() {
        telemetry.addLine("Mode: LeftMode");
        telemetry.update();

        robot.turnAngleRight(30);
        robot.moveRight(ARM_LOAD_POS);
        robot.waitForSystem(50, Systems.WHEELS, Systems.ARM);

        robot.moveClaws(false, true, ClawPositions.CLAWS_FALL, true);

        robot.moveArm(ARM_SCORE_POS);
        robot.moveClaws(false, true, ClawPositions.CLAWS_CLOSED);
        robot.waitForSystem(50, Systems.ARM, Systems.CLAWS);

        robot.turnAngleRight(120, true);

        robot.moveForward(500, true);

        robot.moveClaws(true, false, ClawPositions.CLAWS_FALL, true);

        robot.moveBackward(100);
        robot.moveClaws(true, false, ClawPositions.CLAWS_CLOSED);
        robot.waitForSystem(20, Systems.WHEELS, Systems.CLAWS);

        robot.moveLeft(900, true);

        robot.moveForward(200, true);
    }

    private void useLeftMode() {
        telemetry.addLine("Mode: LeftMode");
        telemetry.update();

        robot.turnAngleLeft(30);
        robot.moveArm(ARM_LOAD_POS);
        robot.waitForSystem(50, Systems.WHEELS, Systems.ARM);

        robot.moveClaws(false, true, ClawPositions.CLAWS_FALL, true);

        robot.moveArm(ARM_SCORE_POS);
        robot.moveClaws(false, true, ClawPositions.CLAWS_CLOSED);
        robot.waitForSystem(50, Systems.ARM, Systems.CLAWS);

        robot.turnAngleLeft(60, true);

        robot.moveRight(500, true);

        robot.moveForward(400, true);

        robot.moveLeft(100, true);

        robot.moveForward(100, true);

        robot.moveClaws(true, false, ClawPositions.CLAWS_FALL, true);

        robot.moveBackward(100, true);

        robot.moveLeft(900);
        robot.moveClaws(true, false, ClawPositions.CLAWS_CLOSED);
        robot.waitForSystem(20, Systems.WHEELS, Systems.CLAWS);

        robot.moveForward(200, true);
    }

    private void useCenterMode() {
        telemetry.addLine("Mode: CenterMode");
        telemetry.update();

        robot.moveForward(480, true);

        robot.moveArm(ARM_LOAD_POS, true);

        robot.moveClaws(false, true, ClawPositions.CLAWS_FALL, true);

        robot.moveArm(ARM_SCORE_POS);
        robot.moveClaws(false, true, ClawPositions.CLAWS_CLOSED);
        robot.waitForSystem(20, Systems.WHEELS, Systems.ARM, Systems.CLAWS);

        robot.moveLeft(800, true);

        robot.turnAngleLeft(90, true);

        robot.moveForward(700, true);

        robot.moveRight(1000, true);

        robot.moveForward(420, true);

        robot.moveClaws(true, false, ClawPositions.CLAWS_FALL, true);

        robot.moveBackward(300, true);

        robot.moveRight(1250);
        robot.moveClaws(true, false, ClawPositions.CLAWS_CLOSED);
        robot.waitForSystem(20, Systems.WHEELS, Systems.CLAWS);

        robot.moveForward(700, true);
    }/*

    public void initWheels(){
        backright = hardwareMap.get(DcMotor.class, "backRight");
        backleft = hardwareMap.get(DcMotor.class, "backLeft");
        frontleft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontright = hardwareMap.get(DcMotor.class, "frontRight");

        backright.setDirection(DcMotorSimple.Direction.FORWARD);
        backleft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontright.setDirection(DcMotorSimple.Direction.FORWARD);

        backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backright.setTargetPosition(0);
        backleft.setTargetPosition(0);
        frontleft.setTargetPosition(0);
        frontright.setTargetPosition(0);

        sleep(10);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(10);
        backright.setPower(1);
        backleft.setPower(1);
        frontleft.setPower(1);
        frontright.setPower(1);
        sleep(10);
    }

    public void initClaws(){
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        leftClaw.resetDeviceConfigurationForOpMode();
        rightClaw.resetDeviceConfigurationForOpMode();
        if (RobotParameters.LEFT_CLAW_INVERTED) {
            leftClaw.setDirection(Servo.Direction.REVERSE);
        } else{
            leftClaw.setDirection(Servo.Direction.FORWARD);
        }
        if (RobotParameters.RIGHT_CLAW_INVERTED) {
            rightClaw.setDirection(Servo.Direction.REVERSE);
        } else{
            rightClaw.setDirection(Servo.Direction.FORWARD);
        }
        rightClaw.setPosition(RobotParameters.RIGHT_CLAW_CLOSED_POSITION);
        leftClaw.setPosition(RobotParameters.LEFT_CLAW_CLOSED_POSITION);

    }

    public void initArm(){
        arm = hardwareMap.get(DcMotor.class, "arm");
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sleep(10);
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.8);
    }

    public void wheelsLinear(int position){
        double rotations = position/537.7;
        int taskTime = Math.abs((int) (1000*(rotations/(312/60))));
        backright.setTargetPosition(backright.getCurrentPosition() + position);
        backleft.setTargetPosition(backleft.getCurrentPosition() + position);
        frontleft.setTargetPosition(frontleft.getCurrentPosition() + position);
        frontright.setTargetPosition(frontright.getCurrentPosition() + position);
        sleep(taskTime+50);
    }

     * Rotates the bot by setting the position of the wheels to a difference. a positive value results in a right rotation, a negative one in a left rotation
     * @param position sets the position difference of the wheels
    public void wheelsRotate(int position){
        double rotations = position/537.7;
        int taskTime = Math.abs((int) (1000*(rotations/(312/60))));
        backright.setTargetPosition(backright.getCurrentPosition() - position);
        backleft.setTargetPosition(backleft.getCurrentPosition() + position);
        frontleft.setTargetPosition(frontleft.getCurrentPosition() + position);
        frontright.setTargetPosition(frontright.getCurrentPosition() - position);
        sleep(taskTime+50);
    }

     * Crab walks to a certain position by setting the position of the wheels t oa difference. positive values to the right negative ones to the left.
     * @param position sets the position difference of the wheels
    public void wheelsCrabWalk(int position){
        double rotations = position/537.7;
        int taskTime = Math.abs((int) (1000*(rotations/(312/60))));
        backright.setTargetPosition(backright.getCurrentPosition() + position);
        backleft.setTargetPosition(backleft.getCurrentPosition() - position);
        frontleft.setTargetPosition(frontleft.getCurrentPosition() + position);
        frontright.setTargetPosition(frontright.getCurrentPosition() - position);
        sleep(taskTime+50);
    }*/

    public DriveModes.PixelPos searchPixel(){
        robot.moveForward(350);
        robot.moveArm(ARM_LOAD_POS);
        robot.waitForSystem(20, Systems.WHEELS, Systems.ARM);

        if (pixelInView()){
            telemetry.speak("I work! I found the pixel in the center. i just don't care about how computers and logic work");
            telemetry.addLine("I found it");
            return DriveModes.PixelPos.CENTER;
        }
        telemetry.addLine("I didn't find it");

        robot.moveArm(ARM_SCORE_POS, true);

        robot.turnLeft(200, true);

        robot.moveArm(ARM_LOAD_POS, true);

        if (pixelInView()){
            robot.moveRight(200);
            robot.moveArm(ARM_SCORE_POS);
            robot.waitForSystem(20, Systems.WHEELS, Systems.ARM);
            return DriveModes.PixelPos.LEFT;

        }else{
            robot.moveRight(200);
            robot.moveArm(ARM_SCORE_POS);
            robot.waitForSystem(20, Systems.WHEELS, Systems.ARM);
            return DriveModes.PixelPos.RIGHT;
        }

    }

    public DriveModes.PixelPos getPixelPos(){
        if (visionPortal == null){
            initTfod();
        }
        //visionPortal.stopStreaming();
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        if (!currentRecognitions.isEmpty()) {
            Recognition highestConfidenceRecognition = null;
            double maxConfidence = 0.0;

            for (Recognition recognition : currentRecognitions) {
                double confidence = recognition.getConfidence();

                if (confidence > maxConfidence) {
                    maxConfidence = confidence;
                    highestConfidenceRecognition = recognition;
                }
            }

            AngleUnit unit = AngleUnit.DEGREES;
            assert highestConfidenceRecognition != null;
            double degrees = highestConfidenceRecognition.estimateAngleToObject(unit);
            double x = (highestConfidenceRecognition.getLeft() + highestConfidenceRecognition.getRight()) / 2 ;
            double y = (highestConfidenceRecognition.getTop()  + highestConfidenceRecognition.getBottom()) / 2 ;
            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", highestConfidenceRecognition.getLabel(), highestConfidenceRecognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", highestConfidenceRecognition.getWidth(), highestConfidenceRecognition.getHeight());
            if (x < 50){
                return DriveModes.PixelPos.LEFT;
            }
            else if(x > 180){
                return DriveModes.PixelPos.CENTER;
            } else {
                return DriveModes.PixelPos.RIGHT;
            }
        } else{
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

    public boolean pixelInView(){
        int i = 0;
        boolean result;
        while (i<500){
            i++;
            List<Recognition> currentRecognitions = tfod.getRecognitions();

            result = currentRecognitions.size() > 0;
            if (result){
                return result;
            }
        }
        List<Recognition> currentRecognitions = tfod.getRecognitions();

        return currentRecognitions.size() > 0;
    }
}