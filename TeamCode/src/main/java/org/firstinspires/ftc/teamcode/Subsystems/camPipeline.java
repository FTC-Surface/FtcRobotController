package org.firstinspires.ftc.teamcode.Subsystems;

import org.firstinspires.ftc.teamcode.Subsystems.Constants;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Core;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;

public class camPipeline extends OpenCvPipeline {

    Mat mainMat = new Mat();
    Mat zone1;
    Mat zone2;
    Mat zone3;

    Scalar zone1RGB;
    Scalar zone2RGB;
    Scalar zone3RGB;

    int zoneNum;
    @Override
    public Mat processFrame(Mat input) {
        mainMat = input.clone();

        zone1 = input.submat(new Rect());
        zone2 = input.submat(new Rect());
        zone3 = input.submat(new Rect());

        zone1RGB = Core.mean(zone1);
        zone2RGB = Core.mean(zone2);
        zone3RGB = Core.mean(zone3);

        if(zone1RGB.val[2] > zone2RGB.val[2]){
            if(zone1RGB.val[2] > zone3RGB.val[3]){
                zoneNum = 1;
            }
            else if(zone1RGB.val[2] < zone3RGB.val[3]){
                zoneNum = 2;
            }
        } else {
                zoneNum = 3;
        }
        return(null);
    }

    public int zone(){
        return zoneNum;
    }
}
