package org.firstinspires.ftc.teamcode.autonomous;



import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

public class FieldTrajectoryPlanner {

    TrajectoryActionBuilder builder;
    Robot robot;
    public FieldTrajectoryPlanner(Robot robot) {
        this.builder = robot.drive.actionBuilder(robot.startingPos);
        this.robot = robot;
    }

    public FieldTrajectoryPlanner dropSpecimen() {
        builder = builder.afterTime(0.1, robot.robotAction(Robot.RobotStates.HIGH_CHAMBER))
                .strafeToLinearHeading(new Vector2d(5*robot.autoPos.xMult, 41.5*robot.autoPos.yMult), Math.toRadians(-90*robot.autoPos.yMult))
                .stopAndAdd(new SequentialAction(
                        new SleepAction(0.5),
                        robot.lift.liftAdjustAction(-400),
                        new SleepAction(.5),
                        robot.claw.clawAction(Claw.ClawStates.OPEN),
                        new SleepAction(0.5)
                ))
                .strafeToLinearHeading(new Vector2d(5*robot.autoPos.xMult, 44*robot.autoPos.yMult), Math.toRadians(-90*robot.autoPos.yMult));

        return this;
    }

    public FieldTrajectoryPlanner pickNeutral(int number) {
        builder = builder
                .afterTime(1, new SequentialAction(robot.lift.liftAction(Lift.LiftStates.FLOOR), new SleepAction(0.25), robot.robotAction(Robot.RobotStates.INTAKE),  new SleepAction(0.1),
                        robot.claw.clawAction(Claw.ClawStates.OPEN)))
                .strafeToLinearHeading(new Vector2d((48+(9.6*number)+(number < 2 ? 0: -10))*robot.autoPos.yMult, (44.5+(number < 2 ? 0 : -4.5))*robot.autoPos.yMult), number < 2 ? Math.toRadians(-90*robot.autoPos.yMult):Math.toRadians(-125*robot.autoPos.yMult) )
                .stopAndAdd(new SequentialAction(

                        new SleepAction(.5+(number < 1 ? 0 : 1.5)),
//                        robot.claw.clawAction(Claw.ClawStates.CLOSE),
                        robot.huskyLens.pickUpAction(robot),
                        new SleepAction(0.75),
                        robot.arm.armAction(Arm.ArmStates.STORED)
                ));
        return this;
    }

    public FieldTrajectoryPlanner dropNet() {
        builder = builder.stopAndAdd(
                        new SequentialAction(
                        robot.lift.liftAction(Lift.LiftStates.HIGH_BASKET),
                        new SleepAction(1.25)))
                .strafeToLinearHeading(new Vector2d(54*robot.autoPos.yMult, 52*robot.autoPos.yMult), Math.toRadians(robot.autoPos.yMult > 0 ? 45 : 225))
                .stopAndAdd(new SequentialAction(
                        robot.arm.armAction(Arm.ArmStates.OUT),
                        new SleepAction(0.75),
                        robot.claw.clawAction(Claw.ClawStates.OPEN),
                        new SleepAction(0.1),
                        robot.claw.clawAction(Claw.ClawStates.CLOSE),
                        new SleepAction(0.1),
                        robot.arm.armAction(Arm.ArmStates.STORED)
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
                .stopAndAdd(new SequentialAction(
                        robot.arm.armAction(Arm.ArmStates.STORED),
                        new SleepAction(0.5),
                        robot.robotAction(Robot.RobotStates.DEFAULT),
                        new SleepAction(2)
                ));
//                .strafeToLinearHeading(new Vector2d(36*robot.autoPos.yMult, 10*robot.autoPos.yMult),  Math.toRadians(robot.autoPos.yMult > 0 ? 180 : 0))
//                .strafeToLinearHeading(new Vector2d(23*robot.autoPos.yMult, 10*robot.autoPos.yMult),  Math.toRadians(robot.autoPos.yMult > 0 ? 180 : 0));
        return this;
    }
}
