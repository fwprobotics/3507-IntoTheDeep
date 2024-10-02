package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.util.TeleopActionRunner;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOpZones extends LinearOpMode {
    TeleopActionRunner actionRunner;
    Robot robot;
    public enum Zones {
        NET (new Pose2d(52, 54, Math.toDegrees(215)), new Pose2d(72, 72, Math.toDegrees(235))),
        TRANSFER (new Pose2d(36, 36, Math.toDegrees(0)), new Pose2d(45, 45, Math.toDegrees(360))),

        SUBMERSIBLE_SIDE (new Pose2d(0, 0, Math.toDegrees(0)), new Pose2d(36, 24, Math.toDegrees(360)));

        public Pose2d minPose;
        public Pose2d maxPose;

        Zones(Pose2d minPose, Pose2d maxPose) {
            this.minPose = minPose;
            this.maxPose = maxPose;
        }

    }

    Zones currentZone;
    Robot.AutoPos autoCorner = Robot.AutoPos.REDNET;

    @Override
    public void runOpMode() throws InterruptedException {
        Drivetrain drivetrain = new Drivetrain(this, hardwareMap, telemetry);
//        Lift lift = new Lift(hardwareMap, telemetry);
         actionRunner = new TeleopActionRunner();
//        Arm arm = new Arm(hardwareMap, telemetry);
         robot = new Robot(hardwareMap, telemetry, Robot.AutoPos.REDNET);
        robot.drive.pose = new Pose2d(-23, -10, 0);
        waitForStart();
        while (!isStopRequested()) {
            if (!gamepad1.touchpad) {
                drivetrain.joystickMovement(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.right_bumper, false, gamepad1.left_bumper);
            }
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
            if (gamepad2.touchpad && !actionRunner.isBusy()) {
                actionRunner.addAction(robot.huskyLens.pickUpAction(robot));
            }

//            if (Math.abs(robot.drive.pose.position.x) > 54 && Math.abs(robot.drive.pose.position.y) > 52 && Math.abs((Math.toDegrees(robot.drive.pose.heading.toDouble())%360)-225) < 10 && robot.currentState == Robot.RobotStates.HIGH_BASKET) {
//                robot.claw.setPosition(Claw.ClawStates.OPEN);
//            }
            processZone(robot.drive.pose);

            robot.drive.updatePoseEstimate();
            telemetry.addData("x", robot.drive.pose.position.x);
            telemetry.addData("y", robot.drive.pose.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(robot.drive.pose.heading.toDouble()));
            actionRunner.update();
            telemetry.update();
        }
    }

    public Zones getCurrentZone(Pose2d currentPos) {
        if (betweenPose(Zones.SUBMERSIBLE_SIDE.minPose, Zones.SUBMERSIBLE_SIDE.maxPose, currentPos)) {
            return Zones.SUBMERSIBLE_SIDE;
        } else if (betweenPose(Zones.TRANSFER.minPose, Zones.TRANSFER.maxPose, currentPos)) {
            return Zones.TRANSFER;
        } else if (betweenPose(Zones.NET.minPose, Zones.NET.maxPose, currentPos)) {
            return Zones.NET;
        } else {
            return currentZone;
        }
    }

    public void processZone(Pose2d currentPos) {
        Zones newZone = getCurrentZone(currentPos);
        if (currentZone != newZone) {
            telemetry.addData("old zone", currentZone);
            if (newZone == Zones.SUBMERSIBLE_SIDE) {
                actionRunner.addAction(robot.robotAction(Robot.RobotStates.INTAKE));
            } else if (newZone == Zones.TRANSFER) {
                if (currentZone == Zones.SUBMERSIBLE_SIDE) {
                    actionRunner.addAction(robot.robotAction(Robot.RobotStates.HIGH_BASKET));
                } else {
                    actionRunner.addAction((robot.robotAction(Robot.RobotStates.DEFAULT)));
                }
            } else if (newZone == Zones.NET) {
                robot.claw.setPosition(Claw.ClawStates.OPEN);
            }

            currentZone = newZone;
        }
        telemetry.addData("current zone", currentZone);
    }

    boolean betweenPose(Pose2d pose1, Pose2d pose2, Pose2d pose) {
        return (pose1.position.x* autoCorner.yMult <= pose.position.x && pose2.position.x*autoCorner.yMult >= pose.position.x) && (pose1.position.y* autoCorner.yMult <= pose.position.y && pose2.position.y* autoCorner.yMult >= pose.position.y) && ((Math.toDegrees(pose1.heading.toDouble())+ (autoCorner.yMult > 0 ? 180 : 0))%360 <= Math.toDegrees(pose.heading.toDouble())%360 && Math.toDegrees(pose2.heading.toDouble())%360 >= Math.toDegrees(pose.heading.toDouble()+ (autoCorner.yMult > 0 ? 180 : 0))%360);
    }
}
