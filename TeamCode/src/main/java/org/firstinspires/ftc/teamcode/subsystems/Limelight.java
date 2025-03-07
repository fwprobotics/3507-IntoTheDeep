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

    public void snapshot() {
        limelight3A.captureSnapshot(String.valueOf(System.currentTimeMillis()));
    }

    public double getRotationalValue() {
        LLResult result = limelight3A.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                return Math.abs(result.getTx() / result.getTy());
            }
        }
        return 10;
    }

    public Wrist.RotateWristStates getWristRotateState() {
        double rot = getRotationalValue();
        if (rot > 1) {
           return Wrist.RotateWristStates.RIGHT;
        } else {
            return Wrist.RotateWristStates.MID;
        }
    }

    public double getTranslationalYValue() {
        LLResult result = limelight3A.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                return result.getTy();
            }
        }
        return 10;
    }
    public double getTranslationalXValue() {
        LLResult result = limelight3A.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                return result.getTx();
            }
        }
        return 10;
    }

    public double getTranslationalValue() {
        double x = getTranslationalXValue();
        double y = getTranslationalYValue();
        return Math.sqrt(Math.pow(x,2)+Math.pow(y,2));
    }


}
