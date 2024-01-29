package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Cam;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;

@Autonomous(name = "AutoOpBlueBottom")
public class BlueLowAuto extends LinearOpMode {

    Cam kam = new Cam();
    Claw claw = new Claw();
    SampleMecanumDrive drive;
    Constants.autoStates currentTraj = Constants.autoStates.idle;

    Pose2d startPose = new Pose2d(12, 59.6, Math.toRadians(270));

    void nextTraj(Constants.autoStates state){
        currentTraj = state;
        telemetry.addData("Trajectory: ", currentTraj);
        telemetry.update();
    }

    public void runOpMode(){
        kam.init(hardwareMap, Constants.cameraColor.blue);
        claw.init(hardwareMap);

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        waitForStart();

        currentTraj = Constants.autoStates.ready;

        telemetry.addLine("Ready");
        telemetry.update();

        int zone = 0;

        while (opModeInInit()) {
            zone = kam.getZone();
            telemetry.addData("Prop Zone", zone);
            telemetry.update();
        }

        waitForStart();

        Pose2d readyPose = new Pose2d();

        if(zone == 0){
            readyPose = new Pose2d(12, 36, Math.toRadians(0));
        } else if(zone == 1){
            readyPose = new Pose2d(12, 36, Math.toRadians(270));
        } else if(zone == 2) {
            readyPose = new Pose2d(12, 36, Math.toRadians(180));
        }

        Trajectory ready = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(readyPose)
                .addDisplacementMarker(() -> {
                    claw.rOpen();
                })
                .build();

        Trajectory park = drive.trajectoryBuilder(ready.end())
                .lineToLinearHeading(new Pose2d(-52, -58.6, Math.toRadians(180)))
                .build();

        while (opModeIsActive()){

            telemetry.addData("Current Traj:", currentTraj);
            telemetry.update();

            switch(currentTraj) {
                case ready:
                    if (!drive.isBusy()) {
                        drive.followTrajectory(ready);
                        nextTraj(Constants.autoStates.idle);
                    }
                    break;
                case idle:
                    break;
            }
        }
    }
}
