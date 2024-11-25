package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Wrist extends Subsystem{

    public enum WristStates {
        DOWN(0.75),
        OUT(0.43);

        public double pos;
        WristStates(double pos) {
            this.pos = pos;
        }
    }

    Servo wristServo;
    double currentPos = 0;
    public Wrist(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry);
        wristServo = hardwareMap.servo.get("wristServo");
    }

    public void setWristState(WristStates state) {
        currentPos = state.pos;
        wristServo.setPosition(currentPos);
    }

    public Action wristAction(WristStates state) {
        return telemetryPacket -> {
            setWristState(state);
            return false;
        };
    }

    //TODO: Model wrist based on arm and attempt manual control
}
