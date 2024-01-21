package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import com.acmerobotics.dashboard.FtcDashboard;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class Cam extends LinearOpMode {
    public OpenCvWebcam kamera;
    public final int camLength = 720;
    public final int camWidth = 1280;
    public CamPipeline pipeline;

    public void init(HardwareMap hardwareMap, Constants.cameraColor color){

        int cameraMonitorViewId =
                hardwareMap
                        .appContext
                        .getResources()
                        .getIdentifier(
                                "cameraMonitorViewId",
                                "id",
                                hardwareMap.appContext.getPackageName());

        kamera = OpenCvCameraFactory.
                    getInstance().
                        createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);

        pipeline = new CamPipeline(color);
        kamera.setPipeline(pipeline);

        kamera.openCameraDeviceAsync(
                new OpenCvCamera.AsyncCameraOpenListener() {
                    @Override
                    public void onOpened() {
                        kamera.startStreaming(camWidth, camLength, OpenCvCameraRotation.UPRIGHT);
                    }

                    @Override
                    public void onError(int errorCode) {
                        telemetry.addData("Camera Initialization: ", errorCode);
                        telemetry.update();
                    }
                });
        FtcDashboard.getInstance().startCameraStream(kamera, 0);
    }

    public int getZone(){
        return pipeline.zone();
    }

    @Override
    public void runOpMode() throws InterruptedException {}
}
