package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15.75)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35.00, 61.00, Math.toRadians(270.0)))
                                .lineToConstantHeading(new Vector2d(-35.00, 34.00))
                                .lineToConstantHeading(new Vector2d(-35.00, 57.00))
                                .lineToConstantHeading(new Vector2d(35.00, 57.00))
                                .splineToSplineHeading(new Pose2d(47.00,35.00,Math.toRadians(0.00)),Math.toRadians(270.00))
                                .lineToConstantHeading(new Vector2d(47.00,10.00))
                                .lineToConstantHeading(new Vector2d(57.00,10.00))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}