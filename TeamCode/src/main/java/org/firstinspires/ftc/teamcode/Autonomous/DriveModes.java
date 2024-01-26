package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;



import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

public class DriveModes {
    public enum StartPos{
        BLUESHORT,
        BLUELONG,
        REDSHORT,
        REDLONG
    }

    public enum GameStrategy{
        DO_NOTHING,
        PUSH_PIXEL,
        PUSH_AND_PLACE,
        PUSH_PLACE_GET_RETURN,
        PUSH_PIXEL_CAM,
        PUSH_AND_PLACE_CAM,
        PUSH_PLACE_GET_RETURN_CAM,
    }

    public enum PixelPos{
        LEFT,
        CENTER,
        RIGHT,
        UNKNOWN
    }
    public static class DriveParams{
        double startRotation;
        double endRotation;
        double PowerCoefficient;
        StartPos startPos;
    }

    private final HardwareMap hardwareMap;
    public PixelPos pixelPos = PixelPos.UNKNOWN;
    public Servo leftClaw;
    public Servo rightClaw;

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


    public DriveModes(StartPos startPos, HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;//FtcOpMode.getInstance().hardwareMap;
        leftClaw = this.hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = this.hardwareMap.get(Servo.class, "rightClaw");
        pixelPos = getPixelPos();

    }

    public PixelPos getPixelPos(){
        if (visionPortal == null){
            initTfod();
        }
        visionPortal.stopStreaming();
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
            if (x < 50){
                return PixelPos.LEFT;
            }
            else if(x > 180){
                return PixelPos.CENTER;
            } else {
                return PixelPos.RIGHT;
            }
        } else{
            return PixelPos.UNKNOWN;
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
                .setAutoStopLiveView(true);

        visionPortal = builder.build();


    } //initTfod

    public boolean isPixelInView(){
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        return currentRecognitions.size() > 0;
    }

}
