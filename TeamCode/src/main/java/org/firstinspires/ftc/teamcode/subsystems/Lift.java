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
        public static double liftStep = 10;
    }

    public enum LiftStates {
        FLOOR(0),
        SPECIMEN(75),
        LOW_CHAMBER (700),
        HANG(800),
        LOW_BASKET (1585),
        HIGH_BASKET (3400);


        public int setPos;
        LiftStates(int setPos) {
            this.setPos = setPos;
        }
    }

    DcMotor leftLift;
    DcMotor rightLift;

    int setPosition = 0;
    public Lift(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry);
        leftLift = hardwareMap.dcMotor.get("leftLiftMotor");
        rightLift = hardwareMap.dcMotor.get("rightLiftMotor");
        leftLift.setDirection(DcMotorSimple.Direction.REVERSE);
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setTargetPosition(setPosition);
        rightLift.setTargetPosition(setPosition);
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftLift.setPower(LiftConfig.liftPower);
        rightLift.setPower(LiftConfig.liftPower);

    }
    //we might need a custom pid loop but yolo
    public void setState(LiftStates state) {
        setPosition = state.setPos;
        leftLift.setTargetPosition(state.setPos);
        rightLift.setTargetPosition(state.setPos);
    }

    public Action liftAction(LiftStates state) {
        return telemetryPacket -> {
            setState(state);
            return false;
        };
    }

    public Action liftAdjustAction(int pos) {
        return telemetryPacket -> {
            setPosition += pos;
            leftLift.setTargetPosition(setPosition);
            rightLift.setTargetPosition(setPosition);
            return false;
        };
    }

    public void manualControl(double power, boolean liftUp, boolean liftDown) {
        setPosition += (int) Math.ceil(power*-LiftConfig.liftStep);
//        if (liftUp) {
//            setPosition = LiftStates.HIGH_BASKET.setPos;
//        }
//        if (liftDown) {
//            setPosition = LiftStates.FLOOR.setPos;
//        }
        leftLift.setTargetPosition(setPosition);
        rightLift.setTargetPosition(setPosition);


        telemetry.addData("lift pos", setPosition);
    }

}
