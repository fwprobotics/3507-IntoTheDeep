package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm extends Subsystem {

    public enum ArmStates {
        STORED (0.05),
        HANG(0.7),
        SAMPLE (0.85),
        OUT (0.9),

        TRANSFER(0); //0.65

        public double setPos;

        ArmStates(double setPos) {
            this.setPos = setPos;
        }
    }

    Servo arm;
    public ArmStates armState;
    public Arm(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry);
        arm = hardwareMap.servo.get("armServo");
    }

    public void setState(ArmStates state) {
        armState = state;
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
