package org.firstinspires.ftc.teamcode.subsystems;

import android.drm.DrmStore;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
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
        FLOOR(0, 0, 1),
        INTAKE(750, 0, 1),
        SPECIMEN(75, 0, 1),
        LOW_CHAMBER (700, 0, 1),
        HANG(800, 0, 1),
        LOW_BASKET (815, 380, 1),
        HIGH_BASKET (2800, 700, 1);


        public int extend;
        public int rotate;
        public double delay;
        LiftStates(int extend, int rotate, double delay) {
            this.extend = extend;
            this.rotate = rotate;
            this.delay = delay;
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
        liftExtend.setDirection(DcMotorSimple.Direction.REVERSE);
        liftRotate.setDirection(DcMotorSimple.Direction.REVERSE);
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
        return (rotatePos/660)*(Math.PI/2);
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

    public Action liftExtendAction(LiftStates state) {
        return  telemetryPacket -> {
            extendSetPos = state.extend;
            liftExtend.setTargetPosition(state.extend);

            return  false;
        };
    }
    public Action liftRotateAction(LiftStates state) {
        return  telemetryPacket -> {
            rotateSetPos = state.rotate;
            liftRotate.setTargetPosition(state.rotate);

            return  false;
        };
    }

    public Action liftAction(LiftStates state) {
        if (rotateSetPos > state.rotate) {
            liftRotate.setPower(0.1);
            return new SequentialAction(
                    liftExtendAction(state),
                    new SleepAction(state.delay),
                    liftRotateAction(state)
            );
        } else {
            liftRotate.setPower(0.5);
            return new SequentialAction(
                    liftRotateAction(state),
                    new SleepAction(state.delay),
                    liftExtendAction(state)
            );
        }
    }
    public Action liftAction(LiftStates state, boolean down) {
        if (down) {
            liftRotate.setPower(0.1);
            return new SequentialAction(
                    liftExtendAction(state),
                    new SleepAction(state.delay),
                    liftRotateAction(state)
            );
        } else {
            liftRotate.setPower(0.5);
            return new SequentialAction(
                    liftRotateAction(state),
                    new SleepAction(state.delay),
                    liftExtendAction(state)
            );
        }
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

    public Action liftPosAction(int extend, int rotate) {
        return telemetryPacket -> {
            extendSetPos = extend;
            rotateSetPos = rotate;
            liftExtend.setTargetPosition(extend);
            liftRotate.setTargetPosition(rotate);
            return false;
        };
    }

    public void manualControl(double powerExtend, double powerRotate) {
        if ((extendSetPos+Math.ceil(powerExtend*-LiftConfig.liftStep))*Math.cos(getAngle()) < 1800 && extendSetPos*Math.cos(getAngle((int) (rotateSetPos+Math.ceil(powerRotate*-LiftConfig.liftStep)))) < 1800) {
            extendSetPos += (int) Math.ceil(powerExtend * -LiftConfig.liftStep);
            rotateSetPos += (int) Math.ceil(powerRotate * -LiftConfig.liftStep);
        }
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
