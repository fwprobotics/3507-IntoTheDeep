package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ArmLiftThingy extends Subsystem{
    DcMotor leftExtendMotor;
    DcMotor rightExtendMotor;

    Servo angleServo;

    int setPosition = 0;
    public ArmLiftThingy(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry);
        leftExtendMotor = hardwareMap.dcMotor.get("leftExtend");
        rightExtendMotor = hardwareMap.dcMotor.get("rightExtend");
        angleServo = hardwareMap.servo.get("angleServo");
        leftExtendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightExtendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

   //     rightExtendMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftExtendMotor.setTargetPosition(setPosition);
        rightExtendMotor.setTargetPosition(setPosition);
        leftExtendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightExtendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftExtendMotor.setPower(Lift.LiftConfig.liftPower);
        rightExtendMotor.setPower(Lift.LiftConfig.liftPower);

    }
    public void manualControl(double power, boolean liftUp, boolean liftDown) {
      //  if (setPosition >= 0) {
            setPosition += (int) Math.ceil(power * -Lift.LiftConfig.liftStep);
     //   }
        leftExtendMotor.setTargetPosition(setPosition);
        rightExtendMotor.setTargetPosition(setPosition);

        if (liftDown) {
            angleServo.setPosition(0);
        }

        if (liftUp) {
            angleServo.setPosition(0.5);
        }


        telemetry.addData("lift pos", setPosition);
    }
}
