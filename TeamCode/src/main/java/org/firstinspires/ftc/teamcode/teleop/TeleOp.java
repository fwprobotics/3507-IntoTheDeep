package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.util.TeleopActionRunner;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Drivetrain drivetrain = new Drivetrain(this, hardwareMap, telemetry);
//        Lift lift = new Lift(hardwareMap, telemetry);
        TeleopActionRunner actionRunner = new TeleopActionRunner();
//        Arm arm = new Arm(hardwareMap, telemetry);
        Robot robot = new Robot(hardwareMap, telemetry, Robot.AutoPos.REDNET);
        waitForStart();
        while (!isStopRequested()) {
            drivetrain.joystickMovement(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.right_bumper, false, gamepad1.left_bumper);
            robot.lift.manualControl(gamepad2.left_stick_y, gamepad2.dpad_up, gamepad2.dpad_down);
            if (gamepad2.dpad_down) {
                actionRunner.addAction( robot.robotAction(Robot.RobotStates.DEFAULT));
            } else if (gamepad2.dpad_up) {
                actionRunner.addAction(robot.robotAction(Robot.RobotStates.HIGH_BASKET));
            } else if (gamepad2.dpad_left) {
                actionRunner.addAction(robot.robotAction(Robot.RobotStates.INTAKE));
            }

            if (gamepad2.a) {
                robot.claw.setPosition(Claw.ClawStates.CLOSE);
            } else if (gamepad2.b) {
                robot.claw.setPosition(Claw.ClawStates.OPEN);
            }

            if (gamepad2.y) {
                robot.wrist.setWristState(Wrist.WristStates.OUT);
            } else if (gamepad2.x) {
                robot.wrist.setWristState(Wrist.WristStates.DOWN);
            }

            actionRunner.update();
            telemetry.update();
        }
    }
}
