package org.firstinspires.ftc.teamcode.Robot.OpenCV;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static org.firstinspires.ftc.teamcode.Robot.RobotParameters.*;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class TeamPropRecognition {
    private static OpenCvCamera       camera;
    private static AutonomousPipeline pipeline;

    /**
     * Constructor for the class TeamPropRecognition that initializes the camera and the pipeline.
     * @param opMode the LinearOpMode that is running the program TODO: testing if it also works with OpMode
     * @param alliance the alliance that the robot is on (Red or Blue)
     */
    public TeamPropRecognition(LinearOpMode opMode, Alliance alliance) {
        pipeline = new AutonomousPipeline();

        pipeline.setAlliance(alliance);

        WebcamName cameraName = opMode.hardwareMap.get(WebcamName.class, CAMERA_STR);

        camera = OpenCvCameraFactory.getInstance().createWebcam(cameraName);

        camera.setPipeline(pipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                opMode.telemetry.addLine("Couldn't find camera");
                opMode.telemetry.update();
            }
        });
    }

    /**
     * @return the position of the team prop
     */
    public TeamPropPosition getTeamPropPosition() {
        return pipeline.getTeamPropPosition();
    }

    /**
     * Stops the camera stream.
     */
    public void stopCameraStream() {
        camera.stopStreaming();
    }
}