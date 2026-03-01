package com.example.meepmeeptesting3;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting3 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(
                        60, 60,
                        Math.toRadians(180),
                        Math.toRadians(180),
                        16.5
                )
                .build();

        myBot.runAction(
                myBot.getDrive().actionBuilder(
                                // Start: (-52, -48) @ 225°
                                new Pose2d(-52, -48, Math.toRadians(225))
                        )
                        .strafeTo(new Vector2d(-36, -34))
                        // Move while keeping heading fixed
                        .strafeTo(new Vector2d(-12, 0))
                        .turnTo(Math.toRadians(180))

                        // Turn to 270°
                        .turnTo(Math.toRadians(270))

                        // Move while keeping 270° heading
                        .strafeTo(new Vector2d(-12, -50))

                        .turnTo(Math.toRadians(225))

                        .splineToConstantHeading(new Vector2d(-24, -45), Math.toRadians(180))

                        .splineToConstantHeading(new Vector2d(-45, -34), Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(-34, -30, Math.toRadians(270)), Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(12, -30, Math.toRadians(270)), Math.toRadians(0))
                        .strafeTo(new Vector2d(12, -50))
                        .strafeTo(new Vector2d(12, -44))
                        .splineToLinearHeading(new Pose2d(-30, -32, Math.toRadians(225)), Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(36,-20, Math.toRadians(270)), Math.toRadians(0))
                        .strafeTo(new Vector2d(36, -50))
                        .splineToLinearHeading(new Pose2d(22, -50, Math.toRadians(270)), Math.toRadians(180))
                        .splineToLinearHeading(new Pose2d(-34, -30, Math.toRadians(225)), Math.toRadians(0))
                        .build()
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}