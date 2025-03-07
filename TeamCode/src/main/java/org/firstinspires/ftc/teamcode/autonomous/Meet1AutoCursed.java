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
public class Meet1AutoCursed extends LinearOpMode {

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
                .dragSpecimen(0)
               // .dragSpecimen(1)
          //      .dragSpecimen(2)
                .humanPlayerPickup(0)
                .dropSpecimen(1)
                .humanPlayerPickup(1)
                .dropSpecimen(2)
                .humanPlayerPickup(2)
                .dropSpecimen(3)
                .humanPlayerPickup(3)
            //    .dropSpecimen(4)
             //  .park()
              //  .ascend()
                .builder.build();

        Actions.runBlocking(robot.robotAction(Robot.RobotStates.DEFAULT));
        robot.wrist.setWristState(Wrist.WristStates.OUT);
        robot.arm.setState(Arm.ArmStates.TRANSFER);

        while (!gamepad1.touchpad) {
            if (gamepad1.y) {
                robot.dropClaw.setPosition(Claw.ClawStates.OPEN);
            } else if (gamepad1.b) {
                robot.dropClaw.setPosition(Claw.ClawStates.CLOSE);
            }
        }

        waitForStart();

        Actions.runBlocking(autoAction);
    }
}
