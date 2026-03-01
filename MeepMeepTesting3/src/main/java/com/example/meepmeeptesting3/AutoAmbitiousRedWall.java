package com.example.meepmeeptesting3;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
public class AutoAmbitiousRedWall {
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
                                new Pose2d(-52, 48, Math.toRadians(135))
                        )
                        .strafeTo(new Vector2d(-50, 46))
                        .splineToLinearHeading(new Pose2d(12, 30, Math.toRadians(90)), Math.toRadians(90))
                        .strafeTo(new Vector2d(12, 46))
                        .strafeTo(new Vector2d(12, 44))
                        .splineToLinearHeading(new Pose2d(-16, 20, Math.toRadians(135)), Math.toRadians(180))
                        .strafeTo(new Vector2d(-14, 18))
                        .splineToLinearHeading(new Pose2d(9, 60, Math.toRadians(115)), Math.toRadians(90))
                        .strafeTo(new Vector2d(12, 58))
                        .splineToLinearHeading(new Pose2d(-16, 20, Math.toRadians(135)), Math.toRadians(180))
                        .strafeTo(new Vector2d(-14, 18))
                        .splineToLinearHeading(new Pose2d(9, 60, Math.toRadians(115)), Math.toRadians(90))
                        .strafeTo(new Vector2d(12, 58))
                        .splineToLinearHeading(new Pose2d(-16, 20, Math.toRadians(135)), Math.toRadians(180))
                        .strafeTo(new Vector2d(-14, 18))
                        .splineToLinearHeading(new Pose2d(9, 60, Math.toRadians(115)), Math.toRadians(90))
                        .strafeTo(new Vector2d(12, 58))
                        .splineToLinearHeading(new Pose2d(-16, 20, Math.toRadians(135)), Math.toRadians(180))

                        .splineToLinearHeading(new Pose2d(-11, 30, Math.toRadians(90)), Math.toRadians(90))
                        .strafeTo(new Vector2d(-11, 46))
                        .splineToLinearHeading(new Pose2d(-52, 22, Math.toRadians(90)), Math.toRadians(90))

                        .build()
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
