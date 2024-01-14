package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Constants;

@Autonomous(name = "AutoOpRedTop")
public class RedHighAuto extends LinearOpMode {
    SampleMecanumDrive drive;
    Constants.autoStates currentTraj = Constants.autoStates.idle;

    Pose2d startPose = new Pose2d(39.5, 59.6, Math.toRadians(270));

    void nextTraj(Constants.autoStates state){

        currentTraj = state;
        telemetry.addData("Trajectory: ", currentTraj);
        telemetry.update();
    }

    public void runOpMode(){
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        Trajectory forward = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(39.5, 55.6, Math.toRadians(180)))
                .build();

        Trajectory park = drive.trajectoryBuilder(forward.end())
                .lineTo(new Vector2d(-54, 55.6))
                .build();

        waitForStart();

        currentTraj = Constants.autoStates.ready;

        telemetry.addLine("Ready");
        telemetry.update();

        while (opModeIsActive()){

            telemetry.addData("Current Traj:", currentTraj);
            telemetry.update();

            switch(currentTraj) {
                case ready:
                    sleep((10000));
                    nextTraj(Constants.autoStates.forward);
                    break;
                case forward:
                    if (!drive.isBusy()) {
                        drive.followTrajectory(forward);
                        nextTraj(Constants.autoStates.park);
                    }
                    break;
                case park:
                    if (!drive.isBusy()) {
                        drive.followTrajectory(park);
                        nextTraj(Constants.autoStates.idle);
                    }
                    break;
                case idle:
                    break;
            }
        }
    }
}
