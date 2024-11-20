package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Hang extends Subsystem{

    CRServo hangServo;

    public Hang(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry);
        this.hangServo = hardwareMap.crservo.get("hangServo");
    }

    public void manualControl(double power) {
        hangServo.setPower(power);
    }

    public Action hangAction(double power) {
        return telemetryPacket -> {
        manualControl(power);
        return false;
        };
    };
}
