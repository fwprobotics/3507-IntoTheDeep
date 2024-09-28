package org.firstinspires.ftc.teamcode.autonomous;



import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Claw;

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
                        new SleepAction(1.5),
                        robot.claw.clawAction(Claw.ClawStates.OPEN),
                        new SleepAction(0.5)
                ))
                .strafeToLinearHeading(new Vector2d(5*robot.autoPos.xMult, 44*robot.autoPos.yMult), Math.toRadians(-90*robot.autoPos.yMult));

        return this;
    }

    public FieldTrajectoryPlanner pickNeutral(int number) {
        builder = builder
                .afterTime(2, robot.robotAction(Robot.RobotStates.INTAKE))
                .strafeToLinearHeading(new Vector2d((48+(10*number))*robot.autoPos.yMult, 45*robot.autoPos.yMult), Math.toRadians(-90*robot.autoPos.yMult))
                .stopAndAdd(new SequentialAction(
                        new SleepAction(3),
                        robot.claw.clawAction(Claw.ClawStates.CLOSE),
                        new SleepAction(1)
                ));
        return this;
    }

    public FieldTrajectoryPlanner dropNet() {
        builder = builder.stopAndAdd(
                        new SequentialAction(
                        robot.robotAction(Robot.RobotStates.HIGH_BASKET),
                        new SleepAction(1.5)))
                .strafeToLinearHeading(new Vector2d(54*robot.autoPos.yMult, 52*robot.autoPos.yMult), Math.toRadians(robot.autoPos.yMult > 0 ? 45 : 225))
                .stopAndAdd(new SequentialAction(
                        new SleepAction(0.75),
                        robot.claw.clawAction(Claw.ClawStates.OPEN),
                        new SleepAction(0.25)
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
}
