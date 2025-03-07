package com.example.meepmeeptesting;


import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setDimensions(16, 16)
                .setConstraints(100, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();


//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-54, -52, Math.toRadians(225)))
//                .setReversed(true)
//        //        .splineToLinearHeading(new Pose2d(-48, -20, 0), Math.toRadians(-90))
//              //          .setReversed(false)
//                .splineToLinearHeading(new Pose2d(-24, -11, Math.toRadians(0)), Math.toRadians(0))
//
//              //  .splineToLinearHeading(new Pose2d(-54, -52, Math.toRadians(180)), Math.toRadians(225))
//                .build());
        Robot robot = new Robot(myBot, Robot.AutoPos.REDHUMAN);
//        myBot.runAction(robot.createTrajectoryPlanner()
//                        .dropSpecimen()
//                        .pickNeutral(0)
//                        .dropNet()
//                .pickNeutral(1)
//                .dropNet()
//                .pickNeutral(2)
//                .dropNet()
//                        .ascend()
//                .builder.build());

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-20, -12, 0))
                        .setReversed(true)
                                .splineToLinearHeading(new Pose2d(-56, -57, Math.toRadians(45)), Math.toRadians(225), new TranslationalVelConstraint(100), new ProfileAccelConstraint(-20, 20))

                .build()
        );

//        myBot.runAction(robot.createTrajectoryPlanner()
//                        .dropSpecimen()
//                .dragSpecimen(0)
//              //  .humanPlayerDrop()
//                .dragSpecimen(1)
//               // .humanPlayerDrop()
//                .dragSpecimen(2)
//                //.humanPlayerDrop()
//                        .humanPlayerPickup()
//                                .dropSpecimen(0)
//                .humanPlayerPickup()
//                .dropSpecimen(1)
//                .humanPlayerPickup()
//                .dropSpecimen(2)
//                .humanPlayerPickup()
//                .dropSpecimen(3)
//                                .park()
//
//
//            //    .ascend()
//                .builder.build());

       // myBot.export("fullMeet1Auto");

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}