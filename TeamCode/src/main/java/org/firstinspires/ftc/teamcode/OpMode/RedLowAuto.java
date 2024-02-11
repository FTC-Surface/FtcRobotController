package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Cam;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.ClawHolder;
import org.firstinspires.ftc.teamcode.Subsystems.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;


public class RedLowAuto extends LinearOpMode {
    Cam kam = new Cam();
    Claw claw = new Claw();
    Arm arm = new Arm();
    Elevator elevator = new Elevator();
    ClawHolder clawHolder = new ClawHolder();

    SampleMecanumDrive drive;
    Constants.autoStates currentTraj = Constants.autoStates.park;

    Pose2d startPose = new Pose2d(12, -59.6, Math.toRadians(90));

    void nextTraj(Constants.autoStates state){
        currentTraj = state;
        telemetry.addData("Trajectory: ", currentTraj);
        telemetry.update();
    }

    public void runOpMode(){
        kam.init(hardwareMap, Constants.cameraColor.red);
        claw.init(hardwareMap);
        arm.init(hardwareMap);
        elevator.init(hardwareMap);
        clawHolder.init(hardwareMap);

        clawHolder.rotate();

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        currentTraj = Constants.autoStates.park;

        int zone = 3;

        while (opModeInInit()) {
            zone = kam.getZone();
            telemetry.addData("Prop Zone", zone);
            telemetry.update();
        }

        waitForStart();

        Vector2d boardVector = new Vector2d();

        Trajectory ready = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(34, 36, Math.toRadians(180)))
                .build();;

        if(zone == 0){
            ready = drive.trajectoryBuilder(startPose)
                    .splineToLinearHeading(new Pose2d(15, -36, Math.toRadians(180)),Math.toRadians(180))
                    .build();

            boardVector = new Vector2d(47.5, -29);
        } else if(zone == 1){
            ready = drive.trajectoryBuilder(startPose)
                    .lineToLinearHeading(new Pose2d(30, -27.5, Math.toRadians(180)))
                    .build();

            boardVector = new Vector2d(47.5, -36);

        } else if(zone == 2) {
            ready = drive.trajectoryBuilder(startPose)
                    .lineToLinearHeading(new Pose2d(37, -36, Math.toRadians(180)))
                    .build();

            boardVector = new Vector2d(47.5, -43);
        }

        TrajectorySequence board = drive.trajectorySequenceBuilder(ready.end())
                .lineToLinearHeading(new Pose2d(boardVector, Math.toRadians(0)))
                .build();

        TrajectorySequence reset = drive.trajectorySequenceBuilder(board.end())
                .lineTo(new Vector2d(47, -57.5))
                .build();

        Trajectory park = drive.trajectoryBuilder(reset.end())
                .lineTo(new Vector2d(57, -56.6))
                .build();

        kam.kamera.stopStreaming();
        kam.kamera.stopRecordingPipeline();

        telemetry.addData("Parking Zone: ", zone);
        telemetry.addLine("Ready");
        telemetry.update();

        while (opModeIsActive()){

            arm.loop();
            drive.update();

            telemetry.addData("Current Traj:", currentTraj);
            telemetry.update();

            switch(currentTraj) {
                case ready:
                    if (!drive.isBusy()) {
                        drive.followTrajectory(ready);
                        claw.rOpen();
                        ElapsedTime timer = new ElapsedTime();
                        timer.reset();
                        while (timer.seconds() < 1){
                        }
                        clawHolder.rotate();
                        nextTraj(Constants.autoStates.board);
                    }
                    break;
                case board:
                    if (!drive.isBusy()) {
                        drive.followTrajectorySequence(board);
                        nextTraj(Constants.autoStates.movedToBoard);
                    }
                    break;
                case movedToBoard:
                    /*arm.setTargetPos(550);
                    while (!arm.isFinished()){
                        arm.loop();
                    }
                    elevator.moveLift(Constants.upDownStates.up, 50);
                    while (!elevator.isFinished()){

                    }
                    ElapsedTime timer = new ElapsedTime();
                    timer.reset();
                    while (timer.seconds() < 0.5){
                    }
                    claw.lOpen();
                    timer.reset();
                    while (timer.seconds() < 0.5){
                    }*/
                    nextTraj(Constants.autoStates.reset);

                case reset:
                    if (!drive.isBusy()) {
                        /*arm.setTargetPos(0);
                        while (!arm.isFinished()){
                            arm.loop();
                        }
                        elevator.moveLift(Constants.upDownStates.up, 0);
                        while (!elevator.isFinished()){
                        }*/
                        drive.followTrajectorySequence(reset);
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
