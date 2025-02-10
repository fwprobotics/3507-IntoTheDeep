package org.firstinspires.ftc.teamcode;

import android.graphics.drawable.Drawable;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.autonomous.FieldTrajectoryPlanner;
import org.firstinspires.ftc.teamcode.pipelines.HuskySampleDetect;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Hang;
import org.firstinspires.ftc.teamcode.subsystems.HorizontalLift;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;

public class Robot {

    public  enum AutoPos {
        REDHUMAN (1, -1),
        REDNET (-1, -1),
        BLUEHUMAN (-1, 1),
        BLUENET (1, 1);

        public int xMult;
        public int yMult;

        AutoPos(int xMult, int yMult) {
            this.xMult = xMult;
            this.yMult = yMult;
        }
    }

    public enum RobotStates {
        DEFAULT (Lift.LiftStates.FLOOR, Arm.ArmStates.TRANSFER),
        INTAKE (Lift.LiftStates.FLOOR, Arm.ArmStates.TRANSFER),
        SPECIMEN (Lift.LiftStates.SPECIMEN, Arm.ArmStates.OUT),
        LOW_CHAMBER (Lift.LiftStates.LOW_CHAMBER, Arm.ArmStates.OUT),
        HIGH_CHAMBER (Lift.LiftStates.LOW_BASKET, Arm.ArmStates.OUT),
        //    LOW_BASKET,
        HIGH_BASKET (Lift.LiftStates.HIGH_BASKET, Arm.ArmStates.OUT),
        HANG (Lift.LiftStates.FLOOR, Arm.ArmStates.HANG);

        Lift.LiftStates liftState;
        Arm.ArmStates armState;

        RobotStates(Lift.LiftStates liftState, Arm.ArmStates armState) {
            this.liftState = liftState;
            this.armState = armState;
        }

    }

    public enum LowerRobotStates {
        INTAKE (HorizontalLift.HLiftStates.MAX, Wrist.WristStates.DOWN, Wrist.RotateWristStates.MID),

        TRANSFER (HorizontalLift.HLiftStates.STORED, Wrist.WristStates.TRANSFER, Wrist.RotateWristStates.MID);

        public HorizontalLift.HLiftStates hLiftState;
        public Wrist.WristStates wristState;
        public Wrist.RotateWristStates rotateWristState;
        LowerRobotStates(HorizontalLift.HLiftStates hLiftState, Wrist.WristStates wristState, Wrist.RotateWristStates rotateWristState) {
            this.hLiftState = hLiftState;
            this.wristState = wristState;
            this.rotateWristState = rotateWristState;
        }
    }

    public Lift lift;
    public Hang hang;
    public HorizontalLift hLift;
    public Arm arm;
    public Wrist wrist;
    public Claw claw;
    public Claw dropClaw;
    public MecanumDrive drive;

    public AutoPos autoPos;

    public RobotStates currentState = RobotStates.DEFAULT;

    public Pose2d startingPos;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, AutoPos autoPos) {
        this.lift = new Lift(hardwareMap, telemetry, false);
       // this.arm = new Arm(hardwareMap, telemetry);
        this.wrist = new Wrist(hardwareMap, telemetry);
        this.claw = new Claw(hardwareMap, telemetry);
        this.dropClaw = new Claw(hardwareMap, telemetry, "dropClawLeft", "dropClawRight");
        this.arm = new Arm(hardwareMap, telemetry);
        this.hLift = new HorizontalLift(hardwareMap, telemetry);
      //  this.huskyLens = new HuskySampleDetect(hardwareMap, telemetry);
        this.startingPos = new Pose2d(8*autoPos.xMult, 63* autoPos.yMult, Math.toRadians(90* autoPos.yMult));
        this.drive = new MecanumDrive(hardwareMap, startingPos);
        this.autoPos = autoPos;

    }


    public Action robotAction(RobotStates state) {
        currentState = state;
        return new SequentialAction(
                //    state == RobotStates.INTAKE ? new SequentialAction(this.wrist.wristAction(Wrist.WristStates.OUT)) : new InstantAction(() -> {}),

                this.lift.liftAction(state.liftState),
                this.arm.armAction(state.armState)
        );
    }

    public Action transferAction() {
        return new SequentialAction(
                this.hLift.hLiftAction(HorizontalLift.HLiftStates.STORED),
                this.wrist.rotateWristState(Wrist.RotateWristStates.MID),
                this.wrist.wristAction(Wrist.WristStates.TRANSFER),
                this.arm.armAction(Arm.ArmStates.TRANSFER),
                this.dropClaw.clawAction(Claw.ClawStates.OPEN),
                new SleepAction(0.7),
                this.dropClaw.clawAction(Claw.ClawStates.CLOSE),
                new SleepAction(0.1),
                this.claw.clawAction(Claw.ClawStates.OPEN),
                new SleepAction(0.1),
                this.hLift.hLiftAction(HorizontalLift.HLiftStates.ABIT),
                //this.arm.armAction(Arm.ArmStates.OUT),
                new SleepAction(0.2),
                this.wrist.wristAction(Wrist.WristStates.OUT));
    }

    public Action transferActionTeleOp() {
        return new SequentialAction(
                this.hLift.hLiftAction(HorizontalLift.HLiftStates.STORED),
                this.wrist.rotateWristState(Wrist.RotateWristStates.MID),
                this.wrist.wristAction(Wrist.WristStates.TRANSFER),
                this.arm.armAction(Arm.ArmStates.TRANSFER),
                this.dropClaw.clawAction(Claw.ClawStates.OPEN),
                new SleepAction(0.5),
                this.dropClaw.clawAction(Claw.ClawStates.CLOSE),
                new SleepAction(0.1),
                this.claw.clawAction(Claw.ClawStates.OPEN));
    }
//    public Action robotAction(RobotStates state) {
//        currentState = state;
//        return new SequentialAction(
//                state == RobotStates.INTAKE ? new SequentialAction(this.wrist.wristAction(Wrist.WristStates.OUT)) : new InstantAction(() -> {}),
//
//                this.lift.liftAction(state.liftState),
//                //      this.arm.armAction(state.armState),
//                this.wrist.wristAction(state.wristState)
//        );
//    }

    public Action lowerRobotAction(LowerRobotStates lowerRobotState) {
        return new SequentialAction(
                hLift.hLiftAction(lowerRobotState.hLiftState),
                wrist.wristAction(lowerRobotState.wristState),
                wrist.rotateWristState(lowerRobotState.rotateWristState)
        );
    }
    public Action lowerRobotAction(LowerRobotStates lowerRobotState, double hLiftPos) {
        return new SequentialAction(
                hLift.hLiftAnalogAction(hLiftPos),
                wrist.wristAction(lowerRobotState.wristState),
                wrist.rotateWristState(lowerRobotState.rotateWristState)
        );
    }

    public void setRobotState(RobotStates state) {
        currentState = state;
        claw.setPosition(Claw.ClawStates.CLOSE);
        //   arm.setState(state.armState);
        //    wrist.setWristState(state.wristState);
        lift.setState(state.liftState);
    }
    public FieldTrajectoryPlanner createTrajectoryPlanner() {
        return new FieldTrajectoryPlanner(this);
    }

    //TODO: husky lens/opencv centering script, combinbed lift, arm, wrist action
}
