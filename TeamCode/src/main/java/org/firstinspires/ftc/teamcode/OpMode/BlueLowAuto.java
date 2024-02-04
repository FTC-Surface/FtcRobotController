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

@Autonomous(name = "AutoOpBlueBottom")
public class BlueLowAuto extends LinearOpMode {
    Cam kam = new Cam();
    Claw claw = new Claw();
    Arm arm = new Arm();
    Elevator elevator = new Elevator();
    ClawHolder clawHolder = new ClawHolder();

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
                .lineToLinearHeading(new Pose2d(34, 34, Math.toRadians(180)))
                .build();;

        if(zone == 0){
            ready = drive.trajectoryBuilder(startPose)
                    .lineToLinearHeading(new Pose2d(37, 34, Math.toRadians(180)))
                    .build();

            boardVector = new Vector2d(47.5, 43);
        } else if(zone == 1){
            ready = drive.trajectoryBuilder(startPose)
                    .lineToLinearHeading(new Pose2d(30, 25.5, Math.toRadians(180)))
                    .build();

            boardVector = new Vector2d(47.5, 36);

        } else if(zone == 2) {
            ready = drive.trajectoryBuilder(startPose)
                    .splineToLinearHeading(new Pose2d(15, 36, Math.toRadians(180)), Math.toRadians(180))
                    .build();

            boardVector = new Vector2d(47, 29);
        }

        TrajectorySequence board = drive.trajectorySequenceBuilder(ready.end())
                .lineToLinearHeading(new Pose2d(boardVector, Math.toRadians(0)))
                .build();

        TrajectorySequence reset = drive.trajectorySequenceBuilder(board.end())
                .lineTo(new Vector2d(47, 57.75))
                .build();

        TrajectorySequence park = drive.trajectorySequenceBuilder(reset.end())
                //.lineTo(new Vector2d(56, 57.75))
                .lineTo(new Vector2d(47, 12))
                .lineTo(new Vector2d(56, 12))
                .build();

        telemetry.addData("Parking Zone: ", zone);
        telemetry.addLine("Ready");
        telemetry.update();

        while (opModeIsActive()){
            kam.kamera.stopStreaming();
            kam.kamera.stopRecordingPipeline();

            claw.close();

            telemetry.addData("Current Traj:", currentTraj);
            telemetry.update();

            switch(currentTraj) {
                /*case ready:
                    if (!drive.isBusy()) {
                        drive.followTrajectory(ready);
                        claw.lOpen();
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
                        /*arm.setTargetPos(550);
                        sleep(300);
                        elevator.moveLift(Constants.upDownStates.up, 50);
                        sleep(300);
                        claw.rOpen();
                        nextTraj(Constants.autoStates.reset);
                    }
                    break;
                case reset:
                    if (!drive.isBusy()) {
                        /*elevator.moveLift(Constants.upDownStates.down, 0);
                        sleep(300);
                        arm.setTargetPos(0);
                        sleep(300);
                        drive.followTrajectorySequence(reset);
                        nextTraj(Constants.autoStates.park);
                    }
                    break;*/
                case park:
                    if (!drive.isBusy()) {
                        drive.followTrajectorySequence(park);
                        nextTraj(Constants.autoStates.idle);
                    }
                    break;
                case idle:
                    break;
            }
        }
    }
}
