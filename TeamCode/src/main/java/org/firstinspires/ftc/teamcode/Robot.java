package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.autonomous.FieldTrajectoryPlanner;
import org.firstinspires.ftc.teamcode.pipelines.HuskySampleDetect;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
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
        DEFAULT (Lift.LiftStates.FLOOR, Arm.ArmStates.STORED, Wrist.WristStates.OUT),
        INTAKE (Lift.LiftStates.FLOOR, Arm.ArmStates.INTAKE, Wrist.WristStates.DOWN),
        SPECIMEN (Lift.LiftStates.SPECIMEN, Arm.ArmStates.OUT, Wrist.WristStates.OUT),
        LOW_CHAMBER (Lift.LiftStates.LOW_CHAMBER, Arm.ArmStates.OUT, Wrist.WristStates.OUT),
        HIGH_CHAMBER (Lift.LiftStates.LOW_BASKET, Arm.ArmStates.OUT, Wrist.WristStates.OUT),
    //    LOW_BASKET,
        HIGH_BASKET (Lift.LiftStates.HIGH_BASKET, Arm.ArmStates.OUT, Wrist.WristStates.OUT),
        HANG (Lift.LiftStates.HANG, Arm.ArmStates.STORED, Wrist.WristStates.OUT);

        Lift.LiftStates liftState;
        Arm.ArmStates armState;
        Wrist.WristStates wristState;

        RobotStates(Lift.LiftStates liftState, Arm.ArmStates armState, Wrist.WristStates wristState) {
            this.liftState = liftState;
            this.armState = armState;
            this.wristState = wristState;
        }

    }

    public Lift lift;
    public Arm arm;
    public Wrist wrist;
    public Claw claw;
    public HuskySampleDetect huskyLens;
    public MecanumDrive drive;

    public AutoPos autoPos;

    public Pose2d startingPos;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, AutoPos autoPos) {
        this.lift = new Lift(hardwareMap, telemetry);
        this.arm = new Arm(hardwareMap, telemetry);
        this.wrist = new Wrist(hardwareMap, telemetry);
        this.claw = new Claw(hardwareMap, telemetry);
        this.huskyLens = new HuskySampleDetect(hardwareMap, telemetry);
        this.startingPos = new Pose2d(10*autoPos.xMult, 63* autoPos.yMult, Math.toRadians(-90* autoPos.yMult));
        this.drive = new MecanumDrive(hardwareMap, startingPos);
        this.autoPos = autoPos;

    }

    public Action robotAction(RobotStates state) {
        return new SequentialAction(
                this.claw.clawAction(Claw.ClawStates.CLOSE),
                this.lift.liftAction(state.liftState),
                this.arm.armAction(state.armState),
                this.wrist.wristAction(state.wristState)
        );
    }

    public FieldTrajectoryPlanner createTrajectoryPlanner() {
        return new FieldTrajectoryPlanner(this);
    }

    //TODO: husky lens/opencv centering script, combinbed lift, arm, wrist action
}
