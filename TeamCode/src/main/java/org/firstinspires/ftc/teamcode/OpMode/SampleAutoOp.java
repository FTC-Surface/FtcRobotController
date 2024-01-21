package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Cam;
import org.firstinspires.ftc.teamcode.Subsystems.Constants;

@Autonomous(name = "TestOpMode")
public class SampleAutoOp extends LinearOpMode {
    Cam cam = new Cam();
    SampleMecanumDrive drive;
    Constants.autoStates currentTraj = Constants.autoStates.idle;

    Pose2d startPose = new Pose2d();

    void nextTraj(Constants.autoStates state){
        currentTraj = state;
    }

    public void runOpMode(){
        cam.init(hardwareMap, Constants.cameraColor.blue);
        drive = new SampleMecanumDrive(hardwareMap);

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
