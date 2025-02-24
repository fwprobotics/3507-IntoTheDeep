package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.util.TeleopActionRunner;

@TeleOp
public class LimelightTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, telemetry, Robot.AutoPos.REDNET);
        telemetry.setMsTransmissionInterval(11);
        TeleopActionRunner actionRunner = new TeleopActionRunner();
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad2.dpad_right && !actionRunner.isBusy()) {
               actionRunner.addAction(robot.autoRotateAction(gamepad2));
            }
            if (gamepad2.touchpad && !actionRunner.isBusy()) {
                actionRunner.addAction(new SequentialAction(robot.hLift.hLiftAnalogAction(0.2), robot.wrist.wristAction(Wrist.WristStates.DOWN), new SleepAction(0.2),robot.autoPickUpAction()));
            }

            if (gamepad2.b) {
                robot.claw.setPosition(Claw.ClawStates.OPEN);
            }
            if (gamepad2.dpad_down) {
                robot.wrist.setWristState(Wrist.WristStates.DOWN);
            }
            telemetry.addData("rotationalValue", robot.limelight.getRotationalValue());
            telemetry.addData("translationalYValue", robot.limelight.getTranslationalYValue());
            telemetry.addData("translationalXValue", robot.limelight.getTranslationalXValue());
            actionRunner.update();
            telemetry.update();
        }
    }
}
