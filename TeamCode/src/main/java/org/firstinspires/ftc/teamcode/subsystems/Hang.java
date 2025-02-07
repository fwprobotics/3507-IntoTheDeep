package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Hang extends Subsystem{
    public enum HangStates {
        IN (1),
        HANG (0.5),
        OUT(0);

        public double pos;

        HangStates(double pos) {
            this.pos = pos;
        }
    }
    public

    Servo hangServo;
    CRServo leftActuator;
    CRServo rightActuator;

    public Hang(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry);
     //   this.hangServo = hardwareMap.servo.get("hangServo");
        this.leftActuator = hardwareMap.crservo.get("leftActuator");
        this.rightActuator = hardwareMap.crservo.get("rightActuator");
    }

    public void manualControl(double power) {
        this.rightActuator.setPower(power);
        this.leftActuator.setPower(power);
    }

    public void setState(HangStates hangState) {
        telemetry.log().add("moving");
        this.rightActuator.setPower(hangState.pos);
        this.leftActuator.setPower(hangState.pos);
    }

    public Action hangAction(double power) {
        return telemetryPacket -> {
     //   manualControl(power);
        return false;
        };
    };
}
