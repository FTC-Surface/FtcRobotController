package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Subsystems.Cam;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.ClawHolder;
import org.firstinspires.ftc.teamcode.Subsystems.Constants;

@Autonomous(name = "Red Low Scrimage")
public class ScrimageRedLowAuto extends LinearOpMode {
    Cam kam = new Cam();
    Claw claw = new Claw();
    ClawHolder clawHolder = new ClawHolder();

    SampleMecanumDrive drive;
    Constants.autoStates currentTraj = Constants.autoStates.idle;

    Pose2d startPose = new Pose2d(12, -59.6, Math.toRadians(90));

    void nextTraj(Constants.autoStates state){
        currentTraj = state;
        telemetry.addData("Trajectory: ", currentTraj);
        telemetry.update();
    }

    public void runOpMode(){
        kam.init(hardwareMap, Constants.cameraColor.blue);
        claw.init(hardwareMap);
        clawHolder.init(hardwareMap);

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        currentTraj = Constants.autoStates.park;

        int zone = 3;

        Trajectory park = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(56, -59.5, Math.toRadians(180)))
                .build();

        telemetry.addData("Parking Zone: ", zone);
        telemetry.addLine("Ready");
        telemetry.update();

        while (opModeIsActive()){
            kam.kamera.stopStreaming();
            kam.kamera.stopRecordingPipeline();

            telemetry.addData("Current Traj:", currentTraj);
            telemetry.update();

            switch(currentTraj) {
                case ready:
                    if (!drive.isBusy()) {
                        clawHolder.reset();
                        ElapsedTime timer = new ElapsedTime();
                        timer.reset();
                        while (timer.seconds() < 1){
                        }
                        claw.close();
                        timer.reset();
                        while (timer.seconds() < 1){
                        }
                        clawHolder.rotate();
                        timer.reset();
                        while (timer.seconds() < 1){
                        }
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
