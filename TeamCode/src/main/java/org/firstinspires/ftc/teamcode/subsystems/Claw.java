package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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
    public Claw(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry);
        rightClaw = hardwareMap.servo.get("rightClawServo");
        leftClaw = hardwareMap.servo.get("leftClawServo");
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



    //TODO: Servo code that moves claw + rr action
}
