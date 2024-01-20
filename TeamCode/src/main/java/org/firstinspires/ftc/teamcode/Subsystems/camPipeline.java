package org.firstinspires.ftc.teamcode.Subsystems;

import org.firstinspires.ftc.teamcode.Subsystems.Constants;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Core;
import org.opencv.core.Scalar;

import java.util.Vector;

public class camPipeline extends OpenCvPipeline {
    int zoneNum;

    private Mat zoneRight, zoneMiddle, zoneLeft;

    private double rightVal, midVal, leftVal;

    static final Rect leftRect = new Rect(
            new Point(0, 100),
            new Point(120, 240));

    static final Rect midRect = new Rect(
            new Point(120, 100),
            new Point(240, 240));
    static final Rect rightRect = new Rect(
            new Point(240, 100),
            new Point(320, 240));

    static final Scalar BLUE = new Scalar(0, 0, 255);

    @Override
    public Mat processFrame(Mat input) {

        Mat mainMat = new Mat();

        Imgproc.cvtColor(input, mainMat, Imgproc.COLOR_RGB2HSV);

        if (mainMat.empty()) {
            zoneNum = 3;
            return input;
        }

        Scalar redLowHSV = new Scalar(0, 70, 50);
        Scalar redHighHSV = new Scalar(0, 255, 255);

        Core.inRange(mainMat, redLowHSV, redHighHSV, mainMat);

        zoneLeft = mainMat.submat(leftRect);
        zoneMiddle = mainMat.submat(midRect);
        zoneRight = mainMat.submat(rightRect);

        leftVal = Core.mean(zoneLeft).val[0];
        midVal = Core.mean(zoneMiddle).val[0];
        rightVal = Core.mean(zoneRight).val[0];

        double firstMax = Math.max(leftVal, rightVal);
        double finalMax = Math.max(firstMax, midVal);

        Imgproc.rectangle(
                input,
                new Point(0, 100),
                new Point(120, 240),
                BLUE,
                1);
        Imgproc.rectangle(
                input,
                new Point(120, 100),
                new Point(240, 240),
                BLUE,
                1);
        Imgproc.rectangle(
                input,
                new Point(240, 100),
                new Point(320, 240),
                BLUE,
                1);


        if(finalMax == leftVal){
            zoneNum = 0;
        } else if(finalMax == midVal) {
            zoneNum = 1;
        }else if(finalMax == rightVal){
            zoneNum = 2;
        } else{
            zoneNum = 3;
        }
        return input;
    }

    public int zone(){
        return zoneNum;
    }
}
