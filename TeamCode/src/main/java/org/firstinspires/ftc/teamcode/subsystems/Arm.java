package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm extends Subsystem {

    public enum ArmStates {
        STORED (0),
        OUT (0.5),

        INTAKE(0.65); //0.85

        public double setPos;

        ArmStates(double setPos) {
            this.setPos = setPos;
        }
    }

    Servo arm;
    public Arm(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry);
        arm = hardwareMap.servo.get("armServo");
    }

    public void setState(ArmStates state) {
        arm.setPosition(state.setPos);
    }

    public Action armAction(ArmStates state) {
        return telemetryPacket -> {
            setState(state);
            return false;
        };
    }

    //TODO: Servo code for flipping arm, rr action that moves arm
}
