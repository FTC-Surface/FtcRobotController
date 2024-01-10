package org.firstinspires.ftc.teamcode.Subsystems;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;

public class camPipeline extends OpenCvPipeline {

    Mat mainMat = new Mat();
    @Override
    public Mat processFrame(Mat input) {

        Imgproc.cvtColor(mainMat, mainMat, Imgproc.COLOR_RGB2HSV);

        return(null);
    }
}
