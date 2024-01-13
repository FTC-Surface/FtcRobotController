package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Cam;
import org.firstinspires.ftc.teamcode.Subsystems.Constants;

public class BlueHighAuto extends LinearOpMode {
    SampleMecanumDrive drive;
    Constants.autoStates currentTraj = Constants.autoStates.idle;

    Pose2d startPose = new Pose2d(39.5, -59.6, Math.toRadians(90));

    void nextTraj(Constants.autoStates state){
        currentTraj = state;
    }

    public void runOpMode(){
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()){

        }
    }
}
