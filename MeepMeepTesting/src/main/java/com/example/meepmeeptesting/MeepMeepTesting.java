package com.example.meepmeeptesting;


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
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();


//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-54, -52, Math.toRadians(225)))
//                .setReversed(true)
//        //        .splineToLinearHeading(new Pose2d(-48, -20, 0), Math.toRadians(-90))
//              //          .setReversed(false)
//                .splineToLinearHeading(new Pose2d(-24, -11, Math.toRadians(0)), Math.toRadians(0))
//
//              //  .splineToLinearHeading(new Pose2d(-54, -52, Math.toRadians(180)), Math.toRadians(225))
//                .build());
        Robot robot = new Robot(myBot, Robot.AutoPos.REDNET);
        myBot.runAction(robot.createTrajectoryPlanner()
                        .dropSpecimen()
                        .pickNeutral(0)
                        .dropNet()
                .pickNeutral(1)
                .dropNet()
                .pickNeutral(2)
                .dropNet()
                        .ascend()
                .builder.build());

        myBot.export("fullMeet1Auto");

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}