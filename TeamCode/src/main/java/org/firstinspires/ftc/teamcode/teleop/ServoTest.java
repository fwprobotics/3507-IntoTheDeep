package org.firstinspires.ftc.teamcode.teleop;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.HorizontalLift;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.util.TeleopActionRunner;

@TeleOp
public class ServoTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, telemetry, Robot.AutoPos.REDNET);
        TeleopActionRunner actionRunner = new TeleopActionRunner();
        waitForStart();
        while (!isStopRequested()) {
            if (gamepad2.dpad_down) {
                actionRunner.addAction(robot.transferAction());
            }
            if (gamepad2.a) {
                robot.claw.setPosition(Claw.ClawStates.OPEN);
            } else if (gamepad2.b) {
                robot.claw.setPosition(Claw.ClawStates.CLOSE);
            }

            if (gamepad2.y) {
                robot.dropClaw.setPosition(Claw.ClawStates.OPEN);
            } else if (gamepad2.x) {
                robot.dropClaw.setPosition(Claw.ClawStates.CLOSE);
            }

            if (gamepad2.dpad_left) {
                robot.hLift.setState(HorizontalLift.HLiftStates.STORED);
            } else if (gamepad2.dpad_right) {
                robot.hLift.setState(HorizontalLift.HLiftStates.MAX);
            }

            if (gamepad2.right_stick_button) {
                robot.wrist.setWristState(Wrist.WristStates.DOWN);
            } else if (gamepad2.left_stick_button) {
                robot.wrist.setWristState(Wrist.WristStates.TRANSFER);
            }

            if (gamepad2.start) {
                robot.wrist.setRotateState(Wrist.RotateWristStates.RIGHT);
            } else if (gamepad2.back) {
                robot.wrist.setRotateState(Wrist.RotateWristStates.LEFT);
            } else if (gamepad2.touchpad) {
                robot.wrist.setRotateState(Wrist.RotateWristStates.MID);
            }

            if (gamepad2.right_bumper) {
                robot.arm.setState(Arm.ArmStates.OUT);
            } else if (gamepad2.left_bumper) {
                robot.arm.setState(Arm.ArmStates.TRANSFER);
            }

//            if (gamepad2.right_trigger > 0.5) {
//                robot.hang.setState(Hang.HangStates.OUT);
//            } else if (gamepad2.left_trigger > 0.5) {
//                robot.hang.setState(Hang.HangStates.IN);
//
//            }

        //    robot.hang.manualControl(gamepad1.right_stick_y);


             robot.hLift.manualControl(gamepad2.right_stick_y);

            actionRunner.update();
            telemetry.addData("claw pos", robot.claw.getPos());
            telemetry.update();
        }
    }
}
