package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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

        double turnDeg = 0;
        Vector2d boardVector = new Vector2d();

        if(zone == 0){
            turnDeg = 90;
            boardVector = new Vector2d(47.5, 29);
        } else if(zone == 1){
            turnDeg = 0;
            boardVector = new Vector2d(47.5, 36);
        } else if(zone == 2) {
            turnDeg = -90;
            boardVector = new Vector2d(47.5, 43);
        }

        TrajectorySequence ready = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(12, 36, Math.toRadians(270)))
                .turn(Math.toRadians(turnDeg))
                .addDisplacementMarker(() -> {
                    claw.rOpen();
                })
                .addDisplacementMarker(() -> {
                    claw.lOpen();
                })
                .build();

        Trajectory board = drive.trajectoryBuilder(ready.end())
                .lineToLinearHeading(new Pose2d(boardVector, Math.toRadians(180)))
                .addDisplacementMarker(() -> {
                    arm.setTargetPos(550);
                })
                .addDisplacementMarker(() -> {
                    elevator.moveLift(Constants.upDownStates.up, 100);
                })
                .addDisplacementMarker(() -> {
                    clawHolder.rotate();
                })
                .addDisplacementMarker(() -> {
                    claw.lOpen();
                })
                .build();

        Trajectory reset = drive.trajectoryBuilder(board.end())
                .addDisplacementMarker(() -> {
                    claw.close();
                })
                .addDisplacementMarker(() -> {
                    clawHolder.reset();
                })
                .addDisplacementMarker(() -> {
                    elevator.moveLift(Constants.upDownStates.down, 0);
                })
                .addDisplacementMarker(() -> {
                    arm.setTargetPos(0);
                })
                .lineTo(new Vector2d(47, 58))
                .build();

        Trajectory park = drive.trajectoryBuilder(reset.end())
                .lineTo(new Vector2d(55, 58))
                .build();

        telemetry.addData("Parking Zone: ", zone);
        telemetry.addLine("Ready");
        telemetry.update();

        while (opModeIsActive()){
            kam.kamera.stopStreaming();
            kam.kamera.stopRecordingPipeline();

            arm.loop();
            claw.close();

            telemetry.addData("Current Traj:", currentTraj);
            telemetry.update();

            switch(currentTraj) {
                case ready:
                    if (!drive.isBusy()) {
                        drive.followTrajectorySequence(ready);
                        nextTraj(Constants.autoStates.idle);
                    }
                    break;
                case idle:
                    break;
            }
        }
    }
}
