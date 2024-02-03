package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Cam;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.ClawHolder;
import org.firstinspires.ftc.teamcode.Subsystems.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;

@Autonomous(name = "AutoOpRedBottom")
public class RedLowAuto extends LinearOpMode {
    Cam kam = new Cam();
    Claw claw = new Claw();
    Arm arm = new Arm();
    Elevator elevator = new Elevator();
    ClawHolder clawHolder = new ClawHolder();
    Constants constants = new Constants();

    SampleMecanumDrive drive;
    Constants.autoStates currentTraj = Constants.autoStates.idle;

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

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        currentTraj = Constants.autoStates.ready;

        int zone = 3;

        claw.close();

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
                    .splineToLinearHeading(new Pose2d(12, -36, Math.toRadians(180)),Math.toRadians(180))
                    .build();

            boardVector = new Vector2d(47.5, -29);
        } else if(zone == 1){
            ready = drive.trajectoryBuilder(startPose)
                    .lineToLinearHeading(new Pose2d(27, -27.5, Math.toRadians(180)))
                    .build();

            boardVector = new Vector2d(47.5, -36);

        } else if(zone == 2) {
            ready = drive.trajectoryBuilder(startPose)
                    .lineToLinearHeading(new Pose2d(34, -36, Math.toRadians(180)))
                    .build();

            boardVector = new Vector2d(47.5, -43);
        }

        TrajectorySequence board = drive.trajectorySequenceBuilder(ready.end())
                .lineTo(boardVector)
                .build();

        TrajectorySequence reset = drive.trajectorySequenceBuilder(board.end())
                .lineTo(new Vector2d(47, -57.5))
                .build();

        Trajectory park = drive.trajectoryBuilder(reset.end())
                .lineTo(new Vector2d(56, -57.5))
                .build();

        telemetry.addData("Parking Zone: ", zone);
        telemetry.addLine("Ready");
        telemetry.update();

        while (opModeIsActive()){
            kam.kamera.stopStreaming();
            kam.kamera.stopRecordingPipeline();

            arm.loop();

            telemetry.addData("Current Traj:", currentTraj);
            telemetry.update();

            sleep(500);

            switch(currentTraj) {
                case ready:
                    sleep(100);
                    if (!drive.isBusy()) {
                        drive.followTrajectory(ready);
                        claw.rOpen();
                        sleep(300);
                        clawHolder.rotate();
                        sleep(300);
                        nextTraj(Constants.autoStates.idle);
                    }
                    break;
                case board:
                    sleep(100);
                    if (!drive.isBusy()) {
                        drive.followTrajectorySequence(board);
                        arm.setTargetPos(550);
                        sleep(300);
                        elevator.moveLift(Constants.upDownStates.up, 50);
                        sleep(300);
                        claw.lOpen();
                        nextTraj(Constants.autoStates.reset);
                    }
                    break;
                case reset:
                    sleep(100);
                    if (!drive.isBusy()) {
                        elevator.moveLift(Constants.upDownStates.down, 0);
                        sleep(300);
                        arm.setTargetPos(0);
                        sleep(300);
                        drive.followTrajectorySequence(reset);
                        nextTraj(Constants.autoStates.park);
                    }
                    break;
                case park:
                    sleep(100);
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
