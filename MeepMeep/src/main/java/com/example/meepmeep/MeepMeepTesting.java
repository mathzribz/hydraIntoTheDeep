package com.example.meepmeep;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 16.5)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(15, -64, 180))
                .setReversed(true)
                .splineTo(new Vector2d(8, -43), Math.toRadians(90))

                .splineToConstantHeading(new Vector2d(33, -35), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(43, -10), Math.toRadians(90))

                .splineToConstantHeading(new Vector2d(50, -58), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(46, -10), Math.toRadians(90))

                .splineToConstantHeading(new Vector2d(62, -58), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(46, -10), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(60, -10), Math.toRadians(90))

                .splineToConstantHeading(new Vector2d(62, -58), Math.toRadians(90))




// Movimentos repetitivos (subindo e descendo)

                .waitSeconds(0.3)
                .strafeTo(new Vector2d(11, -35))
                .waitSeconds(0.3)

                .strafeTo(new Vector2d(38, -58))
                .waitSeconds(0.3)
                .strafeTo(new Vector2d(11, -35))
                .waitSeconds(0.3)

                .strafeTo(new Vector2d(38, -58))
                .waitSeconds(0.3)
                .strafeTo(new Vector2d(11, -35))
                .waitSeconds(0.3)

                .strafeTo(new Vector2d(38, -58))
                .waitSeconds(0.3)
                .strafeTo(new Vector2d(11, -35))
                .waitSeconds(0.3)

// Estacionamento final com spline otimizada

                .strafeToConstantHeading(new Vector2d(42, -62) )



                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}