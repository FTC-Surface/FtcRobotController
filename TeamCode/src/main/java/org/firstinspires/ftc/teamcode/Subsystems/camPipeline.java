package org.firstinspires.ftc.teamcode.Subsystems;

import org.firstinspires.ftc.teamcode.Subsystems.Constants;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Core;
import org.opencv.core.Scalar;

public class camPipeline extends OpenCvPipeline {

    Mat mainMat = new Mat();
    public int zoneNum;

    private Mat zoneRight, zoneMiddle, zoneLeft;

    private double rightVal, midVal, leftVal;

    private Rect leftRect, midRect, rightRect;

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mainMat, Imgproc.COLOR_RGB2HSV);

        Scalar redLowHSV = new Scalar(0, 70, 50);
        Scalar redHighHSV = new Scalar(0, 255, 255);

        Core.inRange(mainMat, redLowHSV, redHighHSV, mainMat);

        zoneLeft = mainMat.submat();
        zoneMiddle = mainMat.submat();
        zoneRight = mainMat.submat();

        leftVal = Core.mean(zoneLeft).val[0];
        midVal = Core.mean(zoneMiddle).val[0];
        rightVal = Core.mean(zoneRight).val[0];

        double firstMax = Math.max(leftVal, rightVal);
        double finalMax = Math.max(firstMax, midVal);

        if(finalMax == leftVal){
            zoneNum = 0;
        } else if(finalMax == rightVal){
            zoneNum = 1;
        } else if(finalMax == midVal){
            zoneNum = 2;
        }

        return input;
    }

    public int zone(){
        return zoneNum;
    }
}
