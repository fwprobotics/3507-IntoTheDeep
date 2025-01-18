package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class HorizontalLift extends Subsystem {

    public enum HLiftStates {
        STORED (0, 0.5),
        ABIT (0.1, 0.4),
        MAX (0.5, 0);

        public double leftPos;
        public double rightPos;

        HLiftStates(double leftPos, double rightPos) {
            this.leftPos = leftPos;
            this.rightPos = rightPos;
        }
    }

    Servo leftHLift;
    Servo rightHLift;

    double currentPos = 0;

    public HorizontalLift(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry);
        leftHLift = hardwareMap.servo.get("leftHLift");
        rightHLift = hardwareMap.servo.get("rightHLift");
    }

    public Action hLiftAction(HLiftStates state) {
        return (telemetryPacket) -> {
            setState(state);
            return false;
        };
    }

    public void setState(HLiftStates state) {
        currentPos = state.leftPos;
        leftHLift.setPosition(state.leftPos);
        rightHLift.setPosition(state.rightPos);
    }

    public void manualControl(double power) {
        currentPos = Math.max(Math.min(currentPos += power*0.05, 0), HLiftStates.MAX.leftPos);
        leftHLift.setPosition(currentPos);
        rightHLift.setPosition(HLiftStates.MAX.leftPos-currentPos);
        telemetry.addData("hLift", currentPos);
    }

}
