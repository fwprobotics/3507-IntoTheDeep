package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;

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
            telemetry.log().add("CLAW POSS"+getPos());
            if (getPos() > 330) {
                robot.setRobotState(Robot.RobotStates.DEFAULT);
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
