package org.firstinspires.ftc.teamcode.subsystems;

import android.drm.DrmStore;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lift extends Subsystem {

    @Config
    public static class LiftConfig {
        public static double liftPower = 1;
        public static double liftStep = 5;
    }

    public enum LiftStates {
        FLOOR(0, 0),
        SPECIMEN(75, 0),
        LOW_CHAMBER (700, 0),
        HANG(800, 0),
        LOW_BASKET (1585, 0),
        HIGH_BASKET (3400, 0);


        public int extend;
        public int rotate;
        LiftStates(int extend, int rotate) {
            this.extend = extend;
            this.rotate = rotate;
        }
    }

    DcMotor liftExtend;
    DcMotor liftRotate;

    int extendSetPos = 0;
    int rotateSetPos = 0;
    public Lift(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry);
        liftExtend = hardwareMap.dcMotor.get("liftExtend");
        liftRotate = hardwareMap.dcMotor.get("liftRotate");
     //   liftExtend.setDirection(DcMotorSimple.Direction.REVERSE);
        liftRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRotate.setTargetPosition(rotateSetPos);
        liftExtend.setTargetPosition(extendSetPos);
        liftRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftRotate.setPower(LiftConfig.liftPower);
        liftExtend.setPower(LiftConfig.liftPower);

    }

    public double getAngle(int rotatePos) {
        return (rotatePos/1000)*(2*Math.PI/3);
    }
    public double getAngle() {
        return getAngle(rotateSetPos);
    }

    public double getXComponent(double angle, int extendPos) {
        return Math.cos(angle)*extendPos;
    }
    public double getYComponent(double angle, int extendPos) {
        return Math.cos(angle)*extendPos;
    }

    public double getXComponent() {
        return getXComponent(getAngle(), extendSetPos);
    }
    public double getYComponent() {
        return getYComponent(getAngle(), extendSetPos);
    }
    //we might need a custom pid loop but yolo
    public void setState(LiftStates state) {
        extendSetPos = state.extend;
        rotateSetPos = state.rotate;
        liftExtend.setTargetPosition(state.extend);
        liftRotate.setTargetPosition(state.rotate);
    }

    public Action liftAction(LiftStates state) {
        return telemetryPacket -> {
            setState(state);
            return false;
        };
    }

    public Action liftAdjustAction(int extend, int rotate) {
        return telemetryPacket -> {
            extendSetPos += extend;
            rotateSetPos += rotate;
            liftExtend.setTargetPosition(extend);
            liftRotate.setTargetPosition(rotate);
            return false;
        };
    }

    public void manualControl(double powerExtend, double powerRotate) {
        extendSetPos += (int) Math.ceil(powerExtend*-LiftConfig.liftStep);
        rotateSetPos += (int) Math.ceil(powerRotate*-LiftConfig.liftStep);
//        if (liftUp) {
//            setPosition = LiftStates.HIGH_BASKET.setPos;
//        }
//        if (liftDown) {
//            setPosition = LiftStates.FLOOR.setPos;
//        }
        liftExtend.setTargetPosition(extendSetPos);
        liftRotate.setTargetPosition(rotateSetPos);


        telemetry.addData("extend lift pos", extendSetPos);
        telemetry.addData("rotate lift pos", rotateSetPos);
        telemetry.addData("x component", getXComponent());
        telemetry.addData("y component", getYComponent());

    }

}
