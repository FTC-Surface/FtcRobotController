package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Cam;

public class AutoOp extends LinearOpMode {
    Cam cam = new Cam();
    public void runOpMode(){
        SampleMecanumDrive drive;

        cam.init(hardwareMap);

        int zone;

        while (opModeInInit()) {
            zone = cam.getZone();
            telemetry.addData("Prop Zone", zone);
            telemetry.update();
        }

        waitForStart();

        cam.kamera.stopStreaming();
        cam.kamera.stopRecordingPipeline();

        while (opModeIsActive() && !isStopRequested()){
            drive = new SampleMecanumDrive(hardwareMap);
        }
    }
}
