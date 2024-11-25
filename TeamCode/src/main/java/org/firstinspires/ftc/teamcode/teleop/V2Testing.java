package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.ArmLiftThingy;

@TeleOp
public class V2Testing extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ArmLiftThingy armLiftThingy = new ArmLiftThingy(hardwareMap, telemetry);

        waitForStart();

        while (!isStopRequested()) {
            armLiftThingy.manualControl(-gamepad1.left_stick_y, gamepad1.a, gamepad1.b);
            telemetry.update();
        }
    }
}
