package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Subsystems.ClawHolder;
import org.firstinspires.ftc.teamcode.Subsystems.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Cam;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;


public class BlueHighAuto extends LinearOpMode {
    Cam kam = new Cam();
    Claw claw = new Claw();
    Arm arm = new Arm();
    Elevator elevator = new Elevator();
    ClawHolder clawHolder = new ClawHolder();
    SampleMecanumDrive drive;
    Constants.autoStates currentTraj = Constants.autoStates.idle;

    Pose2d startPose = new Pose2d(-35, 59.6, Math.toRadians(270));

    void nextTraj(Constants.autoStates state){

        currentTraj = state;
        telemetry.addData("Trajectory: ", currentTraj);
        telemetry.update();
    }

    public void runOpMode(){
        kam.init(hardwareMap, Constants.cameraColor.blue);
        claw.init(hardwareMap);
        arm.init(hardwareMap);
        elevator.init(hardwareMap);
        clawHolder.init(hardwareMap);

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        currentTraj = Constants.autoStates.ready;

        int zone = 3;

        while (opModeInInit()) {
            zone = kam.getZone();
            telemetry.addData("Prop Zone", zone);
            telemetry.update();
        }

        waitForStart();

        Vector2d boardVector = new Vector2d();

        TrajectorySequence ready = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-35, 39, Math.toRadians(0)))
                .build();;

        if(zone == 0){
            ready = drive.trajectorySequenceBuilder(startPose)
                    .splineToLinearHeading(new Pose2d(-35, 39, Math.toRadians(0)), Math.toRadians(0))
                    .turn(Math.toRadians(-90))
                    .build();

            boardVector = new Vector2d(47.5, 43);
        } else if(zone == 1){
            ready = drive.trajectorySequenceBuilder(startPose)
                    .lineToLinearHeading(new Pose2d(-52.5, 28, Math.toRadians(0)))
                    .turn(Math.toRadians(-95))
                    .build();

            boardVector = new Vector2d(47.5, 36);
        } else if(zone == 2) {
            ready = drive.trajectorySequenceBuilder(startPose)
                    .splineToLinearHeading(new Pose2d(-35, 39, Math.toRadians(180)), Math.toRadians(0))
                    .turn(Math.toRadians(90))
                    .build();

            boardVector = new Vector2d(47.5, 29);
        }

        Trajectory spline = drive.trajectoryBuilder(ready.end())
                .splineToLinearHeading(new Pose2d(-21, 0, Math.toRadians(0)), Math.toRadians(0))
                .build();

        Trajectory forward = drive.trajectoryBuilder(spline.end())
                .lineTo(new Vector2d(47.5,0))
                .build();

        Trajectory board = drive.trajectoryBuilder(forward.end())
                .lineTo(boardVector)
                .build();

        Trajectory reset = drive.trajectoryBuilder(board.end())
                .lineTo(new Vector2d(47, 12))
                .build();

        Trajectory park = drive.trajectoryBuilder(reset.end())
                .lineTo(new Vector2d(56, 12))
                .build();

        telemetry.addData("Parking Zone: ", zone);
        telemetry.addLine("Ready");
        telemetry.update();

        while (opModeIsActive()){

            telemetry.addData("Current Traj:", currentTraj);
            telemetry.update();

            switch(currentTraj) {
                case ready:
                    if (!drive.isBusy()) {
                        drive.followTrajectorySequence(ready);
                        claw.lOpen();
                        ElapsedTime timer = new ElapsedTime();
                        timer.reset();
                        while (timer.seconds() < 1){
                        }
                        clawHolder.rotate();
                        nextTraj(Constants.autoStates.spline);
                    }
                    break;
                case spline:
                    sleep(100);
                    if (!drive.isBusy()) {
                        drive.followTrajectory(spline);
                        nextTraj(Constants.autoStates.forward);
                    }
                    break;
                case forward:
                    sleep(100);
                    if (!drive.isBusy()) {
                        drive.followTrajectory(forward);
                        nextTraj(Constants.autoStates.board);
                    }
                    break;
                case board:
                    sleep(100);
                    if (!drive.isBusy()) {
                        drive.followTrajectory(board);
                        nextTraj(Constants.autoStates.reset);
                    }
                    break;
                case reset:
                    sleep(100);
                    if (!drive.isBusy()) {
                        nextTraj(Constants.autoStates.park);
                    }
                    break;
                case park:
                    sleep(100);
                    if (!drive.isBusy()) {
                        drive.followTrajectory(park);
                        clawHolder.reset();
                        nextTraj(Constants.autoStates.idle);
                    }
                    break;
                case idle:
                    break;
            }
        }
    }
}
