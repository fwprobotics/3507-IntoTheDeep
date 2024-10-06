package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Claw;

@Autonomous
public class Meet1Auto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot.AutoPos autoPos = Robot.AutoPos.REDNET;
        while (!gamepad1.a) {
            if (gamepad1.dpad_down) {
                autoPos = Robot.AutoPos.REDNET;
            } else if (gamepad1.dpad_up) {
                autoPos = Robot.AutoPos.REDHUMAN;
            } else if (gamepad1.dpad_left) {
                autoPos = Robot.AutoPos.BLUENET;
            } else if (gamepad1.dpad_right) {
                autoPos = Robot.AutoPos.BLUEHUMAN;
            }
            telemetry.addData("starting pos", autoPos);
            telemetry.update();
        }
        Robot robot = new Robot(hardwareMap, telemetry, autoPos );

        Action autoAction = robot.createTrajectoryPlanner()
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

        Actions.runBlocking(robot.robotAction(Robot.RobotStates.DEFAULT));

        while (!gamepad1.touchpad) {
            if (gamepad1.y) {
                robot.claw.setPosition(Claw.ClawStates.OPEN);
            } else if (gamepad1.b) {
                robot.claw.setPosition(Claw.ClawStates.CLOSE);
            }
        }

        waitForStart();

        Actions.runBlocking(autoAction);
    }
}
