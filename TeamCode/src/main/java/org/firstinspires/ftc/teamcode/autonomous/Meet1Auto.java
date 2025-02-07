package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;

@Autonomous
public class Meet1Auto extends LinearOpMode {
    enum InitStates {
        SELECT,
        PRELOAD,
        READY
    }

    @Override
    public void runOpMode() throws InterruptedException {
        boolean preloadSpec = true;
        InitStates state = InitStates.SELECT;
        Robot robot = new Robot(hardwareMap, telemetry, Robot.AutoPos.REDNET);
        Actions.runBlocking(robot.robotAction(Robot.RobotStates.DEFAULT));
        robot.wrist.setWristState(Wrist.WristStates.OUT);
        robot.arm.setState(Arm.ArmStates.TRANSFER);
        Action autoActionSpec = robot.createTrajectoryPlanner()
                .dropSpecimen()
                .pickNeutral(0)
                .dropNet()
                .pickNeutral(1)
                .dropNet()
                .pickNeutral(2)
                .dropNet()
                .park()
                //  .ascend()
                .builder.build();
        Action autoActionSamp = robot.createTrajectoryPlanner()
                .dropNet()
                .pickNeutral(0)
                .dropNet()
                .pickNeutral(1)
                .dropNet()
                .pickNeutral(2)
                .dropNet()
                .park()
                //  .ascend()
                .builder.build();
        while (!isStopRequested() && state != InitStates.READY) {
            switch (state) {
                case SELECT:
                    if (gamepad1.dpad_up) {
                        preloadSpec = true;
                    } else if (gamepad1.dpad_down) {
                        preloadSpec = false;
                    }
                    if (gamepad1.a) {
                        state = InitStates.PRELOAD;
                    }
                    telemetry.addData("use specimen", preloadSpec);
                    telemetry.update();
                    break;
                case PRELOAD:
                    if (gamepad1.y) {
                        robot.dropClaw.setPosition(Claw.ClawStates.OPEN);
                    } else if (gamepad1.b) {
                        robot.dropClaw.setPosition(Claw.ClawStates.CLOSE);
                    }
                    if (gamepad1.touchpad) {
                        state = InitStates.READY;
                    } else if (gamepad1.back) {
                        state = InitStates.SELECT;
                    }
            }
        }




//        while (!gamepad1.touchpad) {
//            if (gamepad1.y) {
//                robot.dropClaw.setPosition(Claw.ClawStates.OPEN);
//            } else if (gamepad1.b) {
//                robot.dropClaw.setPosition(Claw.ClawStates.CLOSE);
//            }
//        }

        waitForStart();
        if (preloadSpec) {
        Actions.runBlocking(autoActionSpec);
        }
        else {
            Actions.runBlocking(autoActionSamp);
        }
    }
}
