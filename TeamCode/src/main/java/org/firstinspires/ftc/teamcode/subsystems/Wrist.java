package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Wrist extends Subsystem{

    public enum WristStates {
        DOWN(0.35),
        OUT(0.73),
        OUTBACK(1);


        public double pos;
        WristStates(double pos) {
            this.pos = pos;
        }
    }
    public enum RotateWristStates {
        LEFT(0),
        MID(0.5),
        RIGHT(1);

        public double pos;
        RotateWristStates(double pos) {
            this.pos = pos;
        }
    }

    Servo wristServo;
    Servo rotateServo;
    double currentPos = 0;
    double rotateCurrentPos = 0;
    public Wrist(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry);
        wristServo = hardwareMap.servo.get("wristServo");
        rotateServo = hardwareMap.servo.get("rotateServo");
    }

    public void setWristState(WristStates state) {
        currentPos = state.pos;
        wristServo.setPosition(currentPos);
    }

    public void setRotateState(RotateWristStates state) {
        rotateCurrentPos = state.pos;
        rotateServo.setPosition(state.pos);
    }

    public Action wristAction(WristStates state) {
        return telemetryPacket -> {
            setWristState(state);
            return false;
        };
    }

    public Action rotateWristState(RotateWristStates state) {
        return telemetryPacket -> {
            setRotateState(state);
            return false;
        };
    }


    //TODO: Model wrist based on arm and attempt manual control
}
