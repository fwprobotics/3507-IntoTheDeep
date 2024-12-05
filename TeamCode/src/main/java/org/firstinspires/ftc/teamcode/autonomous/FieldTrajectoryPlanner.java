package org.firstinspires.ftc.teamcode.autonomous;



import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

import java.io.SequenceInputStream;

public class FieldTrajectoryPlanner {

    TrajectoryActionBuilder builder;
    Robot robot;
    public FieldTrajectoryPlanner(Robot robot) {
        this.builder = robot.drive.actionBuilder(robot.startingPos);
        this.robot = robot;
    }

    public FieldTrajectoryPlanner dropSpecimen() {
        builder = builder.afterTime(0.1, robot.robotAction(Robot.RobotStates.HIGH_CHAMBER))
                .strafeToLinearHeading(new Vector2d(8*robot.autoPos.xMult, 41.5*robot.autoPos.yMult), Math.toRadians(-90*robot.autoPos.yMult))
                .stopAndAdd(new SequentialAction(
                     //   new SleepAction(0.5),
                   //     robot.lift.liftAdjustAction(-400, 0),
                      //  new SleepAction(.5),
                        robot.claw.clawAction(Claw.ClawStates.OPEN),
                        new SleepAction(0.5)
                ))
                .strafeToLinearHeading(new Vector2d(8*robot.autoPos.xMult, 44*robot.autoPos.yMult), Math.toRadians(-90*robot.autoPos.yMult));

        return this;
    }

    public FieldTrajectoryPlanner dropSpecimen(int i) {
        builder = builder.stopAndAdd(robot.robotAction(Robot.RobotStates.HIGH_CHAMBER))
                .strafeToLinearHeading(new Vector2d((0+(i*2))*robot.autoPos.xMult, (40+(i > 0 ? -0.5 : 0))*robot.autoPos.yMult), Math.toRadians(-90*robot.autoPos.yMult))

                .strafeToLinearHeading(new Vector2d((0+(i*2))*robot.autoPos.xMult, (33+(i > 0 ? -0.5 : 0))*robot.autoPos.yMult), Math.toRadians(-90*robot.autoPos.yMult))

                .stopAndAdd(new SequentialAction(
                        new SleepAction(0.1),
                     //   robot.lift.liftAdjustAction(-400, 0),
                        new SleepAction(.1),
                        robot.claw.clawAction(Claw.ClawStates.OPEN),
                        new SleepAction(0.1)
                ))
                .strafeToLinearHeading(new Vector2d(5*robot.autoPos.xMult, 44*robot.autoPos.yMult), Math.toRadians(-90*robot.autoPos.yMult));

        return this;
    }

    public FieldTrajectoryPlanner pickNeutral(int number) {
        builder = builder
                .afterTime(1, new SequentialAction(
                        new SleepAction(0.2),
                        robot.robotAction(Robot.RobotStates.INTAKE, true),
                        new SleepAction(0.2),
                        robot.claw.clawAction(Claw.ClawStates.OPEN)))
                .strafeToLinearHeading(new Vector2d((48+(9.6*number)+(number < 2 ? 0: -9.6))*robot.autoPos.yMult, (44.5+(number < 2 ? 0 : -4.5))*robot.autoPos.yMult), number < 2 ? Math.toRadians(-90*robot.autoPos.yMult):Math.toRadians(-125*robot.autoPos.yMult) )
                .stopAndAdd(new SequentialAction(

                        new SleepAction(.5+(number < 1 ? 0 : 1.5)),
                        robot.claw.clawAction(Claw.ClawStates.CLOSE),
                    //    robot.huskyLens.pickUpAction(robot),
                        new SleepAction(0.75)
                   //     robot.arm.armAction(Arm.ArmStates.STORED)
                ));
        return this;
    }

    public FieldTrajectoryPlanner dragSpecimen(int number) {
        if (number == 0) {
            builder = builder
                    .stopAndAdd(robot.robotAction(Robot.RobotStates.DEFAULT))
                    .splineToConstantHeading(new Vector2d(-(35+(number*9.6))*robot.autoPos.yMult, (45)*robot.autoPos.yMult), Math.toRadians(90))
                    .stopAndAdd(new SleepAction(0.1))
                    .splineToConstantHeading(new Vector2d(-(35+(number*9.6))*robot.autoPos.yMult, (16)*robot.autoPos.yMult), Math.toRadians(90));

        }
        builder = builder
//                .afterTime(1, new SequentialAction(robot.lift.liftAction(Lift.LiftStates.FLOOR), new SleepAction(0.25), robot.robotAction(Robot.RobotStates.INTAKE),  new SleepAction(0.1),
//                        robot.claw.clawAction(Claw.ClawStates.OPEN)))
                //    .strafeToLinearHeading(new Vector2d(-(40)*robot.autoPos.yMult, (41+(number < 2 ? 0 : -4.5))*robot.autoPos.yMult), Math.toRadians(45-(15*number)))
                .splineToConstantHeading(new Vector2d(-(43+(number*9.6))*robot.autoPos.yMult, (12)*robot.autoPos.yMult), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(40+(number*9.6), 45*robot.autoPos.yMult), Math.toRadians(-90))

                .stopAndAdd(new SequentialAction(

                        //       new SleepAction(.25+(number < 1 ? 0 : 0.25)),
//                      //  robot.claw.clawAction(Claw.ClawStates.CLOSE),
//                        //  robot.huskyLens.pickUpAction(robot),
                        //     new SleepAction(.1)
                        //  robot.arm.armAction(Arm.ArmStates.STORED)
                ));
        return this;
    }

    public FieldTrajectoryPlanner pickSpecimen(int number) {
        builder = builder
                .stopAndAdd(new SequentialAction(robot.lift.liftPosAction(400+(200*number), 0)))
                .strafeToLinearHeading(new Vector2d(-(36)*robot.autoPos.yMult, (48+(number < 2 ? 0 : -4.5))*robot.autoPos.yMult), Math.toRadians(45-(10*number)))
                .stopAndAdd(new SequentialAction(

                   //     new SleepAction(.25+(number < 1 ? 0 : 0.25)),
                          robot.claw.clawAction(Claw.ClawStates.CLOSE)
                        //  robot.huskyLens.pickUpAction(robot),
                   //     new SleepAction(.1)
                        //  robot.arm.armAction(Arm.ArmStates.STORED)
                ));
        return this;
    }

    public FieldTrajectoryPlanner humanPlayerDrop() {
        builder = builder
                //    .afterTime(1, new SequentialAction(robot.robotAction(Robot.RobotStates.INTAKE), new SleepAction(0.25), robot.claw.clawAction(Claw.ClawStates.OPEN)))
                .strafeToLinearHeading(new Vector2d(36, 49*robot.autoPos.yMult), Math.toRadians(-40))
                .stopAndAdd(new SequentialAction(robot.claw.clawAction(Claw.ClawStates.OPEN), new SleepAction(0.1)));
//                .strafeToLinearHeading(new Vector2d(54, 39*robot.autoPos.yMult), Math.toRadians(90*robot.autoPos.yMult))
//              //  .stopAndAdd(new SequentialAction(robot.robotAction(Robot.RobotStates.SPECIMEN), robot.claw.clawAction(Claw.ClawStates.OPEN),new SleepAction(1.25)))
//                .strafeToLinearHeading(new Vector2d(54, 54*robot.autoPos.yMult), Math.toRadians(90*robot.autoPos.yMult))
        // .stopAndAdd(new SequentialAction(robot.claw.clawAction(Claw.ClawStates.CLOSE), new SleepAction(0.5), robot.robotAction(Robot.RobotStates.DEFAULT)));

        return this;
    }

    public FieldTrajectoryPlanner humanPlayerPickup() {
        builder = builder
                .stopAndAdd(robot.robotAction(Robot.RobotStates.DEFAULT))
                .strafeToLinearHeading(new Vector2d(36, 54*robot.autoPos.yMult), Math.toRadians(90*robot.autoPos.yMult))
                .stopAndAdd(new SequentialAction(robot.claw.clawAction(Claw.ClawStates.CLOSE), new SleepAction(0.1)));
        return this;
    }

//    public FieldTrajectoryPlanner humanPlayerPick() {
//
//    }
    public FieldTrajectoryPlanner dropNet() {
        builder = builder.stopAndAdd(
                        new SequentialAction(
                        robot.robotAction(Robot.RobotStates.HIGH_BASKET),
                        new SleepAction(1.25)))
                .strafeToLinearHeading(new Vector2d(54*robot.autoPos.yMult, 52*robot.autoPos.yMult), Math.toRadians(robot.autoPos.yMult > 0 ? 45 : 45))
                .stopAndAdd(new SequentialAction(
                     //   robot.arm.armAction(Arm.ArmStates.OUT),
                        new SleepAction(0.75),
                        robot.claw.clawAction(Claw.ClawStates.OPEN)
                    //    robot.arm.armAction(Arm.ArmStates.STORED)
                ));
        return this;
    }

    public FieldTrajectoryPlanner ascend() {
        builder = builder
                .afterTime(1, robot.robotAction(Robot.RobotStates.DEFAULT))
                .strafeToLinearHeading(new Vector2d(36*robot.autoPos.yMult, 10*robot.autoPos.yMult),  Math.toRadians(robot.autoPos.yMult > 0 ? 180 : 0))
                .strafeToLinearHeading(new Vector2d(23*robot.autoPos.yMult, 10*robot.autoPos.yMult),  Math.toRadians(robot.autoPos.yMult > 0 ? 180 : 0));
        return this;
    }

    public FieldTrajectoryPlanner park() {
        builder = builder
                .afterTime(0.1, new SequentialAction(
                  //      robot.arm.armAction(Arm.ArmStates.STORED),
                        new SleepAction(0.5),
                        robot.robotAction(Robot.RobotStates.HANG)
                   //     robot.hang.hangAction(1)
                ))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-23, -11, Math.toRadians(0)), Math.toRadians(0), new TranslationalVelConstraint(100), new ProfileAccelConstraint(-20, 20))
                ;
//                .strafeToLinearHeading(new Vector2d(36*robot.autoPos.yMult, 10*robot.autoPos.yMult),  Math.toRadians(robot.autoPos.yMult > 0 ? 180 : 0))
//                .strafeToLinearHeading(new Vector2d(23*robot.autoPos.yMult, 10*robot.autoPos.yMult),  Math.toRadians(robot.autoPos.yMult > 0 ? 180 : 0));
        return this;
    }
}
