package org.firstinspires.ftc.teamcode.Orientation;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import static org.firstinspires.ftc.teamcode.Robot.RobotParameters.*;

import java.util.Locale;
import java.util.List;


public class AprilTagRecognition {
    private       AprilTagProcessor aprilTag;
    public static LinearOpMode      currOpMode;
    private VisionPortal visionPortal;

    /**
     * Constructor for AprilTagRecognition
     * @param opMode The LinearOpMode that the robot is running TODO: test if it works with OpMode as well
     */
    public AprilTagRecognition(LinearOpMode opMode) {
        currOpMode = opMode;
        initAprilTag();
    }

    /**
     * Initializes the AprilTag recognition system.
     */
    private void initAprilTag() {
        if (VERBOSE >= 4) currOpMode.telemetry.addLine("start initializing AprilTagRecognition");
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        if (USE_WEBCAM) {
            builder.setCamera(currOpMode.hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        builder.setCameraResolution(new Size(CAMERA_WIDTH, CAMERA_HEIGHT));

        if (ENABLE_LIVE_VIEW) {
            builder.enableLiveView(true);


            // Set the stream format; MJPEG uses less bandwidth than default YUY2.
            builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

            // Choose whether or not LiveView stops if no processors are enabled.
            // If set "true", monitor shows solid orange screen if no processors enabled.
            // If set "false", monitor shows camera view without annotations.
            builder.setAutoStopLiveView(true);
        }
        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        VisionPortal visionPortal = builder.build();


        // Disable or re-enable the processor at any time.
        visionPortal.setProcessorEnabled(aprilTag, true);
        if (VERBOSE >= 4) currOpMode.telemetry.addLine("AprilTagRecognition initialized");


    }   // end method initAprilTag()

    /**
     * stops the camera stream
     */
    public void stopStreaming() {
        visionPortal.stopStreaming();
    }

    /**
     * resumes the camera stream
     */
    public void resumeStreaming() {
        visionPortal.resumeStreaming();
    }

    /**
     * closes the vision portal
     */
    public void close_visionPortal() {
        visionPortal.close();
    }

    /**
     * this should return the 2D position of the robot in relation to the april tag (in cm and degrees)
     *
     * @return A Position2D representing the position of the robot.
     */
    public Position2D getRobotPosition() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        int count = currentDetections.size();
        if (VERBOSE >= 4)
            currOpMode.telemetry.addLine(String.format(Locale.US, "found %d aprilTags", count));
        if (count == 0) return null;

        double sumX = 0, sumY = 0, sumAngle = 0;
        for (AprilTagDetection detection : currentDetections) {
            Position2D aprilTagPose;
            try {
                aprilTagPose = APRIL_TAG_POSES[detection.id - 1];
            } catch (ArrayIndexOutOfBoundsException e) {
                aprilTagPose = null;
                if (VERBOSE >= 2) {
                    currOpMode.telemetry.addLine("we recognized an april tag that we don't have in our list. this is a problem.");
                }
            }
            if (detection.metadata != null && aprilTagPose != null) {
                double angle = detection.ftcPose.bearing - detection.ftcPose.yaw + aprilTagPose.angle;
                double y_offset = Math.cos(Math.toRadians(angle)) * detection.ftcPose.range;
                double x_offset = Math.sin(Math.toRadians(angle)) * detection.ftcPose.range;

                sumX += x_offset + aprilTagPose.x;
                sumY += y_offset + aprilTagPose.y;
                sumAngle += angle;
                if (VERBOSE >= 4)
                    currOpMode.telemetry.addLine(String.format(Locale.US, "Bot-Pos: X:%6.2f, Y:%6.2f, rot:%6.2f", x_offset, y_offset, angle));
            }
        }
            double averageX = sumX / count;
            double averageY = sumY / count;
            double averageAngle = sumAngle / count; //TODO: add measures to reassure that all positions are valid. if something is more that 10% or so different from the other april tags, leave it out.
            if (VERBOSE >= 4)
                currOpMode.telemetry.addLine(String.format(Locale.US, "Bot-Pos: X:%6.2f, Y:%6.2f, rot:%6.2f", averageX, averageY, averageAngle));
            return new Position2D(averageX, averageY, averageAngle);
        }
}
