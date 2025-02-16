package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Limelight extends Subsystem {
    Limelight3A limelight3A;
    public Limelight(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry);
        limelight3A = hardwareMap.get(Limelight3A.class, "Ethernet Device");
        limelight3A.pipelineSwitch(0);
        limelight3A.start();
    }

    public double getRotationalValue() {
        LLResult result = limelight3A.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                return Math.abs(result.getTx() / result.getTy());
            }
        }
        return 0;
    }

    public Wrist.RotateWristStates getWristRotateState() {
        double rotationalValue = getRotationalValue();
        if (rotationalValue > 1.5) {
           return Wrist.RotateWristStates.MID;
        } else if (rotationalValue > 1.1) {
            return Wrist.RotateWristStates.RIGHT;
        } else {
            return Wrist.RotateWristStates.LEFT;
        }
    }

    public double getTranslationalValue() {
        LLResult result = limelight3A.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                return result.getTy();
            }
        }
        return 0;
    }


}
