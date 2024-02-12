package com.example.meepmeeptest;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTest {
    public static void main(String args[]){
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-34, -59.6, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(-44, 0, Math.toRadians(0)))
                                .lineTo(new Vector2d(56, -12))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}

/*
                        Blue top
                        drive.trajectorySequenceBuilder(new Pose2d(-35, 59.6, Math.toRadians(270)))
                                .splineToLinearHeading(new Pose2d(-35, 39, Math.toRadians(0)), Math.toRadians(0))
                                .turn(Math.toRadians(-90))
                                .splineToLinearHeading(new Pose2d(-35, 0, Math.toRadians(0)), Math.toRadians(0))
                                .lineTo(new Vector2d(47,0))
                                .lineTo(new Vector2d(47, 43))
                                .lineTo(new Vector2d(47, 12))
                                .lineTo(new Vector2d(56, 12))
                                .build()

                       Blue bottom
                       drive.trajectorySequenceBuilder(new Pose2d(12, 59.6, Math.toRadians(270)))
                                .lineToLinearHeading(new Pose2d(34, 36, Math.toRadians(180)))
                                .lineTo(new Vector2d(47.5, 43))
                                .lineTo(new Vector2d(47, 57.75))
                                .lineTo(new Vector2d(56, 57.75))
                                .build()

                     Red Bottom
                     drive.trajectorySequenceBuilder(new Pose2d(12, -59.6, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(34, -36, Math.toRadians(180)))
                                .lineTo(new Vector2d(47.5, -29))
                                .lineTo(new Vector2d(47, -57.75))
                                .lineTo(new Vector2d(56, -57.75))
                                .build()


 */