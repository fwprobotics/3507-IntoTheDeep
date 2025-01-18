package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Hang;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.util.TeleopActionRunner;
import org.firstinspires.ftc.teamcode.util.ToggleButton;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOpZones extends LinearOpMode {
    TeleopActionRunner actionRunner;

    //TeleopActionRunner driverRunner;
    Robot robot;
    public enum Zones {
        NET (new Pose2d(50, 50, Math.toRadians(215)), new Pose2d(72, 72, Math.toRadians(235))),
        TRANSFER (new Pose2d(36, 36, Math.toRadians(0)), new Pose2d(72, 72, Math.toRadians(360))),

        SUBMERSIBLE_SIDE (new Pose2d(0, 0, Math.toRadians(0)), new Pose2d(30, 24, Math.toRadians(360))),
        HUMAN (new Pose2d(-72, 40, Math.toRadians(0)), new Pose2d(-36, 72, Math.toRadians(360))),
        SPECIMEN(new Pose2d(-24, 12, Math.toRadians(75)), new Pose2d(24, 48, Math.toRadians(105))),
        DEFAULT_AUD(new Pose2d(36, -24, Math.toRadians(0)), new Pose2d(72, 24, Math.toRadians(360))),
        DEFAULT_DRIVER(new Pose2d(-72, 24, Math.toRadians(0)), new Pose2d(24, 72, Math.toRadians(360)));

        public Pose2d minPose;
        public Pose2d maxPose;

        Zones(Pose2d minPose, Pose2d maxPose) {
            this.minPose = minPose;
            this.maxPose = maxPose;
        }

    }

    Zones currentZone = Zones.TRANSFER;
    Robot.AutoPos autoCorner = Robot.AutoPos.REDNET;
    boolean usedZoneBased = true;

    @Override
    public void runOpMode() throws InterruptedException {
        Drivetrain drivetrain = new Drivetrain(this, hardwareMap, telemetry);
//        Lift lift = new Lift(hardwareMap, telemetry);
         actionRunner = new TeleopActionRunner();
//        Arm arm = new Arm(hardwareMap, telemetry);
         robot = new Robot(hardwareMap, telemetry, Robot.AutoPos.REDNET, true);
         Hang hang = new Hang(hardwareMap, telemetry);
        robot.drive.pose = new Pose2d(-23, -11, Math.toRadians(0));
        ToggleButton zoneBased = new ToggleButton(true);
        ToggleButton clawClose = new ToggleButton(false);
        waitForStart();
        actionRunner.addAction(robot.robotAction(Robot.RobotStates.DEFAULT));
        while (!isStopRequested()) {
            if (!gamepad1.touchpad ) {
                drivetrain.joystickMovement(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.right_bumper, false, gamepad1.left_bumper);
            }
            robot.lift.manualControl(gamepad2.left_stick_y, gamepad2.dpad_up, gamepad2.dpad_down, gamepad2.touchpad);
            if (gamepad2.dpad_down) {
                actionRunner.addAction( robot.robotAction(Robot.RobotStates.DEFAULT));
            } else if (gamepad2.dpad_up) {
                actionRunner.addAction(robot.robotAction(Robot.RobotStates.HIGH_BASKET));
            } else if (gamepad2.dpad_left) {
                actionRunner.addAction(robot.robotAction(Robot.RobotStates.INTAKE));
            }  else if (gamepad2.dpad_right) {
                actionRunner.addAction(robot.robotAction(Robot.RobotStates.HIGH_CHAMBER));
            }
            clawClose.toggle(gamepad2.a);
            if (clawClose.newPress) {
                robot.claw.setPosition(Claw.ClawStates.CLOSE);
                actionRunner.addAction(new SequentialAction(new SleepAction(0.25), robot.claw.autoClawAction(robot)));
            } else if (gamepad2.b) {
                robot.claw.setPosition(Claw.ClawStates.OPEN);
            }

            if (gamepad2.y) {
                robot.wrist.setWristState(Wrist.WristStates.OUT);
            } else if (gamepad2.x) {
                robot.wrist.setWristState(Wrist.WristStates.DOWN);
            }
            if (gamepad1.touchpad && (!actionRunner.isBusy() || gamepad1.y)) {
                actionRunner.addAction(robot.huskyLens.pickUpAction(robot));
            }

//            if (Math.abs(robot.drive.pose.position.x) > 54 && Math.abs(robot.drive.pose.position.y) > 52 && Math.abs((Math.toDegrees(robot.drive.pose.heading.toDouble())%360)-225) < 10 && robot.currentState == Robot.RobotStates.HIGH_BASKET) {
//                robot.claw.setPosition(Claw.ClawStates.OPEN);
//            }
            hang.manualControl(-gamepad2.right_stick_y);

            zoneBased.toggle(gamepad2.right_bumper);

            if (zoneBased.newPress) {
                usedZoneBased = !usedZoneBased;
            }
            robot.drive.updatePoseEstimate();

            if (usedZoneBased) {
                processZone(robot.drive.pose);
                gamepad2.setLedColor(255, 0, 0, 10);
            } else {
                gamepad2.setLedColor(0, 255, 0, 10);

            }
            telemetry.addData("zone based?", zoneBased.state);
            telemetry.addData("x", robot.drive.pose.position.x);
            telemetry.addData("y", robot.drive.pose.position.y);
            double norm_ang = Math.toDegrees(robot.drive.pose.heading.toDouble()) < 0 ? 360 + Math.toDegrees(robot.drive.pose.heading.toDouble()): Math.toDegrees(robot.drive.pose.heading.toDouble());
            telemetry.addData("heading (deg)", norm_ang);
            telemetry.addData("clawPos", robot.claw.getPos());
            actionRunner.update();
            telemetry.update();
        }
    }

    public Zones getCurrentZone(Pose2d currentPos) {
        if (betweenPose(Zones.SUBMERSIBLE_SIDE.minPose, Zones.SUBMERSIBLE_SIDE.maxPose, currentPos)) {
            return Zones.SUBMERSIBLE_SIDE;
        }else if (betweenPose(Zones.NET.minPose, Zones.NET.maxPose, currentPos)) {
            return Zones.NET;
        } else if (betweenPose(Zones.TRANSFER.minPose, Zones.TRANSFER.maxPose, currentPos)) {
            return Zones.TRANSFER;
        }  else if (betweenPose(Zones.HUMAN.minPose, Zones.HUMAN.maxPose, currentPos)) {
            return Zones.HUMAN;
        }
     else if (betweenPose(Zones.SPECIMEN.minPose, Zones.SPECIMEN.maxPose, currentPos)) {
        return Zones.SPECIMEN;
    } else if (betweenPose(Zones.DEFAULT_AUD.minPose, Zones.DEFAULT_AUD.maxPose, currentPos)) {
            return Zones.DEFAULT_AUD;
        } else if (betweenPose(Zones.DEFAULT_DRIVER.minPose, Zones.DEFAULT_DRIVER.maxPose, currentPos)) {
            return Zones.DEFAULT_DRIVER;
        }

        else {
            return currentZone;
        }
    }

    public void processZone(Pose2d currentPos) {
        Zones newZone = getCurrentZone(currentPos);
        if (currentZone != newZone) {
            telemetry.addData("old zone", currentZone);
            if (newZone == Zones.SUBMERSIBLE_SIDE) {
//                actionRunner.addAction(new SequentialAction(
//                        robot.robotAction(
//                        Robot.RobotStates.INTAKE),
//                        new SleepAction(0.5),
//                        robot.claw.clawAction(Claw.ClawStates.OPEN)));
            } else if (newZone == Zones.TRANSFER) {
                if (currentZone == Zones.SUBMERSIBLE_SIDE || currentZone == Zones.DEFAULT_AUD || currentZone == Zones.DEFAULT_DRIVER) {
                    actionRunner.addAction(robot.robotAction(Robot.RobotStates.HIGH_BASKET));
                } else {
                    actionRunner.addAction((robot.robotAction(Robot.RobotStates.DEFAULT)));
                }
            } else if (newZone == Zones.NET) {
                if (currentZone == Zones.TRANSFER) {
                //    robot.claw.setPosition(Claw.ClawStates.OPEN);
                }
            } else if (newZone == Zones.HUMAN) {
                actionRunner.addAction(robot.robotAction(Robot.RobotStates.SPECIMEN));
            } else if (newZone == Zones.SPECIMEN) {
                actionRunner.addAction(robot.robotAction(Robot.RobotStates.HIGH_CHAMBER));
            } else if (newZone == Zones.DEFAULT_AUD || newZone == Zones.DEFAULT_DRIVER) {
                actionRunner.addAction((robot.robotAction(Robot.RobotStates.DEFAULT)));
            }

            currentZone = newZone;
        }
        telemetry.addData("current zone", currentZone);
    }

    boolean betweenPose(Pose2d pose1, Pose2d pose2, Pose2d pose) {
      //  double norm_ang = Math.toDegrees(pose.heading.toDouble()) < 0 ? 360 + Math.toDegrees(pose.heading.toDouble()): Math.toDegrees(pose.heading.toDouble());
   //     telemetry.log().add("min angle: "+getNormAngle(pose1));
        //     telemetry.log().add("min pose: "+pose1.position.times(autoCorner.yMult)+ " max pose: "+pose2.position.times(autoCorner.yMult)+" currentPose "+pose);
        return (pose1.position.x <= pose.position.x* autoCorner.yMult && pose2.position.x >= pose.position.x* autoCorner.yMult) &&
                (pose1.position.y <= pose.position.y* autoCorner.yMult && pose2.position.y>= pose.position.y* autoCorner.yMult) &&
                ((getNormAngle(pose1) <= (getNormAngle(pose)+ (autoCorner.yMult > 0 ? 180 : 0))%360 && getNormAngle(pose2) >= (getNormAngle(pose)+ (autoCorner.yMult > 0 ? 180 : 0))%360))
                ;

        // && ((Math.toDegrees(pose1.heading.toDouble())+ (autoCorner.yMult > 0 ? 180 : 0))%360 <= Math.toDegrees(pose.heading.toDouble())%360 && Math.toDegrees(pose2.heading.toDouble())%360 >= Math.toDegrees(pose.heading.toDouble()+ (autoCorner.yMult > 0 ? 180 : 0))%360
    }

    double getNormAngle(Pose2d pose) {
        return Math.toDegrees(pose.heading.toDouble()) < 0 ? 360 + Math.toDegrees(pose.heading.toDouble()) : Math.toDegrees(pose.heading.toDouble());
    }
}
