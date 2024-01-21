package org.firstinspires.ftc.teamcode.Subsystems;

import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Core;
import org.opencv.core.Scalar;

public class CamPipeline extends OpenCvPipeline {
    int zoneNum;

    Scalar highHSV;
    Scalar lowHSV;

    private Mat zoneRight, zoneMiddle, zoneLeft;

    private double rightVal, midVal, leftVal;

    static final Rect leftRect = new Rect(
            new Point(0, 150),
            new Point(420, 720));

    static final Rect midRect = new Rect(
            new Point(420, 150),
            new Point(840, 720));
    static final Rect rightRect = new Rect(
            new Point(840, 150),
            new Point(1280, 720));

    static final Scalar BLUE = new Scalar(0, 0, 255);
    static final Scalar RED = new Scalar(255, 0, 0);
    static final Scalar GREEN = new Scalar(0, 255, 0);

    Constants.cameraColor teamColor;

    public CamPipeline(Constants.cameraColor color){
        teamColor = color;
    }

    @Override
    public Mat processFrame(Mat input) {

        Mat mainMat = new Mat();

        Imgproc.cvtColor(input, mainMat, Imgproc.COLOR_RGB2HSV);

        if (mainMat.empty()) {
            zoneNum = 3;
            return input;
        }

        if(teamColor == Constants.cameraColor.red){
            lowHSV = new Scalar(160, 50, 50);
            highHSV = new Scalar(180, 255, 255);
        } else if (teamColor == Constants.cameraColor.blue){
            lowHSV = new Scalar(100, 150, 0);
            highHSV = new Scalar(140, 255, 255);
        }

        Core.inRange(mainMat, lowHSV, highHSV, mainMat);

        zoneLeft = mainMat.submat(leftRect);
        zoneMiddle = mainMat.submat(midRect);
        zoneRight = mainMat.submat(rightRect);

        Imgproc.rectangle(
                input,
                leftRect,
                BLUE,
                1);
        Imgproc.rectangle(
                input,
                midRect,
                RED,
                1);
        Imgproc.rectangle(
                input,
                rightRect,
                GREEN,
                1);

        leftVal = Core.mean(zoneLeft).val[0];
        midVal = Core.mean(zoneMiddle).val[0];
        rightVal = Core.mean(zoneRight).val[0];

        double firstMax = Math.max(leftVal, rightVal);
        double finalMax = Math.max(firstMax, midVal);

        if(finalMax == leftVal){
            zoneNum = 0;
        } else if(finalMax == midVal) {
            zoneNum = 1;
        }else if(finalMax == rightVal){
            zoneNum = 2;
        } else{
            zoneNum = 3;
        }

        mainMat.copyTo(input);
        mainMat.release();

        return input;
    }

    public int zone(){
        return zoneNum;
    }
}
