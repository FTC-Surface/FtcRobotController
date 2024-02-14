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

@Autonomous(name = "Blue Top Scrimage")
public class ScrimageBlueTopAuto extends LinearOpMode {
    Claw claw = new Claw();
    ClawHolder clawHolder = new ClawHolder();

    SampleMecanumDrive drive;
    Constants.autoStates currentTraj = Constants.autoStates.idle;

    Pose2d startPose = new Pose2d(-34, 59.6, Math.toRadians(270));

    void nextTraj(Constants.autoStates state){
        currentTraj = state;
    }

    public void runOpMode(){
        claw.init(hardwareMap);
        clawHolder.init(hardwareMap);

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        currentTraj = Constants.autoStates.ready;

        waitForStart();

        clawHolder.rotate();
        claw.open();

        Trajectory forward = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-44, 0, Math.toRadians(0)))
                .build();

        Trajectory park = drive.trajectoryBuilder(forward.end())
                .lineTo(new Vector2d(56, 12))
                .build();

        while (opModeIsActive()){
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
                        nextTraj(Constants.autoStates.forward);
                    }
                    break;
                case forward:
                    if(!drive.isBusy()){
                        drive.followTrajectory(forward);
                        nextTraj(Constants.autoStates.park);
                    }
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
