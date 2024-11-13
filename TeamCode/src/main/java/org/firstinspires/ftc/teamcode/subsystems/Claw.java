package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;

import java.util.concurrent.atomic.AtomicBoolean;

public class Claw extends Subsystem{

    public enum ClawStates {
        OPEN (0.5, 0),
        CLOSE (0.0, 0.5);

        public double rightSetPos;
        public double leftSetPos;

        ClawStates(double setPos, double leftSetPos) {
            this.rightSetPos = setPos;
            this.leftSetPos = leftSetPos;
        }
    }

    Servo rightClaw;
    Servo leftClaw;
    AnalogInput analogInput;
    public Claw(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry);
        rightClaw = hardwareMap.servo.get("rightClawServo");
        leftClaw = hardwareMap.servo.get("leftClawServo");
        analogInput = hardwareMap.analogInput.get("clawPos");
    }
    public void setPosition(ClawStates state) {
        rightClaw.setPosition(state.rightSetPos);
        leftClaw.setPosition(state.leftSetPos);
    }

    public Action clawAction(ClawStates state) {
        return (telemetryPacket) -> {
            setPosition(state);
            return false;
        };

    }

    public Action autoClawAction(Robot robot) {
        return (telemetryPacket) -> {
            telemetry.addData("CLAW POSS", getPos());
            if (getPos() > 325) {
                robot.setRobotState(Robot.RobotStates.DEFAULT);
            } else {
                setPosition(ClawStates.OPEN);
            }
            return false;
        };
    }

    public Action autoClawActionSuper(Robot robot) {
        Action action = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToLinearHeading(new Vector2d(-48, -12), Math.toRadians(0))
                .stopAndAdd(robot.robotAction(Robot.RobotStates.HIGH_BASKET))
                .strafeToLinearHeading(new Vector2d(-54, -52), Math.toRadians(225))
                .stopAndAdd(robot.claw.clawAction(ClawStates.OPEN))
                .strafeToLinearHeading(new Vector2d(-48, -12), Math.toRadians(0))
                .stopAndAdd(robot.robotAction(Robot.RobotStates.DEFAULT))
                .strafeToLinearHeading(new Vector2d(-24, -8), Math.toRadians(0))
                .build();
        AtomicBoolean isDriving = new AtomicBoolean(false);
        return (telemetryPacket) -> {
            telemetry.addData("CLAW POSS", getPos());
            if (getPos() > 315 || isDriving.get()) {
                if (!isDriving.get()) {
                    robot.setRobotState(Robot.RobotStates.DEFAULT);
                    isDriving.set(true);
                }
                return action.run(telemetryPacket);
            } else {
                setPosition(ClawStates.OPEN);
            }
            return false;
        };
    }

    public double getPos() {
        return analogInput.getVoltage()/3.3*360;
    }



    //TODO: Servo code that moves claw + rr action
}
