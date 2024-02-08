package org.firstinspires.ftc.teamcode.Robot.OpenCV;

import static org.firstinspires.ftc.teamcode.Robot.RobotParameters.*;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class AutonomousPipeline extends OpenCvPipeline {
    private static final Scalar           RED              = new Scalar(255, 0, 0);
    private static final Scalar           GREEN            = new Scalar(0, 255, 0);
    private static final Scalar           BLUE             = new Scalar(0, 0, 255);
    private volatile     TeamPropPosition teamPropPosition;

    //
    // Region points
    //
    private static final int   REGION_WIDTH  = 50;
    private static final int   REGION_HEIGHT = 50;
    private static final Point region1PointA = new Point(100, 100);
    private static final Point region1PointB = new Point(region1PointA.x + REGION_WIDTH, region1PointA.y + REGION_HEIGHT);
    private static final Point region2PointA = new Point(400, 100);
    private static final Point region2PointB = new Point(region2PointA.x + REGION_WIDTH, region2PointA.y + REGION_HEIGHT);
    private static final Point region3PointA = new Point(700, 100);
    private static final Point region3PointB = new Point(region3PointA.x + REGION_WIDTH, region3PointA.y + REGION_HEIGHT);

    //
    // Regions
    //
    private static final Rect region1 = new Rect(region1PointA, region1PointB);
    private static final Rect region2 = new Rect(region2PointA, region2PointB);
    private static final Rect region3 = new Rect(region3PointA, region3PointB);

    //
    // Materials
    //
    private static Mat region1Cb, region2Cb, region3Cb;
    private static final Mat YCrCb = new Mat();
    private static final Mat Cb    = new Mat();

    public void inputToCb(Mat input) {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, Cb, 2);
    }

    @Override
    public void init(Mat firstFrame) {
        inputToCb(firstFrame);

        region1Cb = Cb.submat(region1);
        region2Cb = Cb.submat(region2);
        region3Cb = Cb.submat(region3);
    }

    @Override
    public Mat processFrame(Mat input) {

        Imgproc.rectangle(input, region1PointA, region1PointB, BLUE, 2);
        Imgproc.rectangle(input, region2PointA, region2PointB, BLUE, 2);
        Imgproc.rectangle(input, region3PointA, region3PointB, BLUE, 2);

        inputToCb(input);

        int avg1 = (int) Core.mean(region1Cb).val[0];
        int avg2 = (int) Core.mean(region2Cb).val[0];
        int avg3 = (int) Core.mean(region3Cb).val[0];

        int max = Math.max(Math.max(avg1, avg2), avg3);

        if (max == avg1) {
            teamPropPosition = TeamPropPosition.LEFT;

            Imgproc.rectangle(
                    input,
                    region1PointA,
                    region1PointB,
                    GREEN,
                    -1);
        } else if (max == avg2) {
            teamPropPosition = TeamPropPosition.CENTER;

            Imgproc.rectangle(
                    input,
                    region2PointA,
                    region2PointB,
                    GREEN,
                    -1);
        } else {
            teamPropPosition = TeamPropPosition.RIGHT;

            Imgproc.rectangle(
                    input,
                    region3PointA,
                    region3PointB,
                    GREEN,
                    -1);
        }

        return input;
    }

    public TeamPropPosition getTeamPropPosition() {
        return teamPropPosition;
    }
}
