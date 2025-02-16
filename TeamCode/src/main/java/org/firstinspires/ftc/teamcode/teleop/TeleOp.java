package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.HorizontalLift;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.util.TeleopActionRunner;
import org.firstinspires.ftc.teamcode.util.ToggleButton;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Drivetrain drivetrain = new Drivetrain(this, hardwareMap, telemetry);
//        Lift lift = new Lift(hardwareMap, telemetry);
        TeleopActionRunner actionRunner = new TeleopActionRunner();
//        Arm arm = new Arm(hardwareMap, telemetry);
        ToggleButton clawClose = new ToggleButton(false);
        ToggleButton specimenState = new ToggleButton(false);
        ToggleButton wristRotateToggle = new ToggleButton(false);
        ElapsedTime elapsedTime = new ElapsedTime();
        Robot robot = new Robot(hardwareMap, telemetry, Robot.AutoPos.REDNET);
        waitForStart();
        robot.hLift.setState(HorizontalLift.HLiftStates.STORED);
        elapsedTime.reset();
        while (!isStopRequested()) {
            drivetrain.joystickMovement(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.right_bumper, false, gamepad1.left_bumper);
            robot.lift.manualControl(gamepad2.left_stick_y, gamepad2.dpad_up, gamepad2.dpad_down, gamepad2.touchpad);
            robot.hLift.manualControl(gamepad1.right_trigger-gamepad1.left_trigger);
            specimenState.toggle(gamepad2.touchpad);
            if (gamepad2.dpad_down) {
                actionRunner.addAction( robot.robotAction(Robot.RobotStates.DEFAULT));
            } else if (gamepad2.dpad_up && !actionRunner.isBusy()) {
                //reserved for side pickup
                if (!specimenState.state) {
                    actionRunner.addAction(robot.robotAction(Robot.RobotStates.HIGH_BASKET));
                } else {
                    actionRunner.addAction(robot.robotAction(Robot.RobotStates.HIGH_CHAMBER));
                }
            } else if (gamepad2.dpad_left) {
              //  actionRunner.addAction(robot.robotAction(Robot.RobotStates.INTAKE));
                robot.wrist.setWristState(Wrist.WristStates.DOWN);
            } else if (gamepad2.dpad_right) {
                robot.wrist.setWristState(Wrist.WristStates.OUT);
                //override the automated transfer
             //   actionRunner.addAction(robot.robotAction(Robot.RobotStates.HIGH_CHAMBER));
            }

//            if (gamepad2.right_bumper) {
//                actionRunner.addAction( robot.robotAction(Robot.RobotStates.DEFAULT));
//            } else if (gamepad2.left_bumper) {
//                if (!specimenState.state) {
//                    actionRunner.addAction(robot.robotAction(Robot.RobotStates.HIGH_BASKET));
//                } else {
//                    actionRunner.addAction(robot.robotAction(Robot.RobotStates.HIGH_CHAMBER));
//                }
//            }

//            if (gamepad2.a) {
//                robot.claw.setPosition(Claw.ClawStates.CLOSE);
//            } else if (gamepad2.b) {
//                robot.dropClaw.setPosition(Claw.ClawStates.OPEN);
//            }
//            wristRotateToggle.toggle(gamepad2.x);
//            if (gamepad2.y) {
//                robot.wrist.setRotateState(Wrist.RotateWristStates.MID);
//            } else if (wristRotateToggle.newPress) {
//                if (robot.wrist.rotateWristState == Wrist.RotateWristStates.MID) {
//                    robot.wrist.setRotateState(Wrist.RotateWristStates.LEFT);
//                } else if (robot.wrist.rotateWristState == Wrist.RotateWristStates.LEFT) {
//                    robot.wrist.setRotateState(Wrist.RotateWristStates.RIGHT);
//                } else {
//                    robot.wrist.setRotateState(Wrist.RotateWristStates.LEFT);
//                }
//            }
            if (gamepad2.x) {
                robot.wrist.setRotateState(Wrist.RotateWristStates.LEFT);
            } else if (gamepad2.y) {
                robot.wrist.setRotateState(Wrist.RotateWristStates.RIGHT);
            } else if (gamepad2.right_bumper) {
                robot.wrist.setRotateState(Wrist.RotateWristStates.MID);
            }
            clawClose.toggle(gamepad2.a);
            if (clawClose.newPress) {
         //       telemetry.log().add("ALERT1");
                robot.claw.setPosition(Claw.ClawStates.CLOSE);
                actionRunner.addAction(new SequentialAction(new SleepAction(0.25), robot.claw.autoClawAction(robot, gamepad2)));
            } else if (gamepad2.b) {
                //does this work?
                if (robot.arm.armState == Arm.ArmStates.TRANSFER) {
                    robot.claw.setPosition(Claw.ClawStates.OPEN);
                } else {
                    robot.dropClaw.setPosition(Claw.ClawStates.OPEN);
                }
            }

            if (gamepad1.right_stick_button) {
                robot.hLift.setState(HorizontalLift.HLiftStates.MAX);
            } else if (gamepad1.left_stick_button) {
                robot.hLift.setState(HorizontalLift.HLiftStates.STORED);
            }

            if (specimenState.state) {
                gamepad2.runLedEffect(new Gamepad.LedEffect.Builder().addStep(255, 0, 0, 100).setRepeating(true).build());
            } else {
                gamepad2.runLedEffect(new Gamepad.LedEffect.Builder().addStep(0, 255, 0, 100).setRepeating(true).build());
            }

            if (Math.abs(elapsedTime.milliseconds() - 90000) < 500) {
                gamepad1.rumbleBlips(3);
                gamepad2.rumbleBlips(3);
            }
        //    robot.hang.manualControl(gamepad2.right_trigger-gamepad2.left_trigger);
            telemetry.addData("claw pos", robot.claw.getPos());
            actionRunner.update();
            telemetry.update();
        }
    }
}
