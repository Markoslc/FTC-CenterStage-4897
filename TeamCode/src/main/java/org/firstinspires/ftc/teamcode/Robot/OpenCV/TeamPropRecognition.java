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

    public TeamPropPosition getTeamPropPosition(){
        return pipeline.getTeamPropPosition();
    }

    public void stopCameraStream(){
        camera.stopStreaming();
    }
}