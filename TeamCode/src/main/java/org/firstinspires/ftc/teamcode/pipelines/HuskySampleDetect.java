package org.firstinspires.ftc.teamcode.pipelines;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Claw;

public class HuskySampleDetect {

    @Config
    public static class HuskyConfig {
        public static double aspectLower = 0;
        public static double aspectUpper = 10;
        public static double minArea = 100;

    }

    public class Sample {

    }

    HuskyLens huskyLens;

    Telemetry telemetry;

    public HuskySampleDetect(HardwareMap hardwareMap, Telemetry telemetry) {
        this.huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens");
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        this.telemetry = telemetry;
    }

    public HuskyLens.Block[] readBlocks() {
        return huskyLens.blocks();
    }

//    public Sample getNearestSample() {
//        HuskyLens.Block[] detectedBlocks = readBlocks();
//        HuskyLens.Block selectedBlock = null;
//        for (HuskyLens.Block block : detectedBlocks) {
//            //if it meets required params
//            if (block.width*block.height >= HuskyConfig.minArea && block.width/block.height >= HuskyConfig.aspectLower && block.width/block.height <= HuskyConfig.aspectUpper) {
//                if (selectedBlock != null) {
//
//                }
//            }
//        }
//    }

    public HuskyLens.Block getNearestSample() {
        HuskyLens.Block[] detectedBlocks = readBlocks();
        HuskyLens.Block selectedBlock = null;
        for (HuskyLens.Block block : detectedBlocks) {
            //if it meets required params
            if (block.width * block.height >= HuskyConfig.minArea && block.width / block.height >= HuskyConfig.aspectLower && block.width / block.height <= HuskyConfig.aspectUpper) {
               selectedBlock = block;
            }
        }
      //  telemetry.log().add(selectedBlock.toString());
        return selectedBlock;

    }

    public Action pickUpAction(Robot robot) {
        Vector2d center = new Vector2d(160, 120);
        double timeStarted = System.currentTimeMillis();
        int offsetX = 0;
        int offsetY = 30;
        return telemetryPacket -> {
            HuskyLens.Block block = getNearestSample();
            if (block != null) {
                telemetry.log().add("Distance x: " + (Math.hypot(- block.x + center.x , - block.y + ( center.y  + offsetY))));
//                telemetry.log().add("Distance y: " + (Math.abs(block.y - 120)));
//                telemetry.log().add("Distance from center: " + (Math.hypot(block.x - 160, block.y - 120)));
                if ((Math.hypot(- block.x + center.x , - block.y + ( center.y  + offsetY)) < 30) || System.currentTimeMillis()-timeStarted > 750) {
                    robot.drive.setDrivePowers(new PoseVelocity2d(
                            new Vector2d(0, 0), 0
                    ));
                    robot.claw.setPosition(Claw.ClawStates.CLOSE);
                    return false;
                }
                robot.drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d((center.y -  block.y ) * 0.005, (center.x - block.x) * 0.005), 0
                ));
            } else {
                telemetry.log().add("nothing "+(System.currentTimeMillis()-timeStarted));
                if (System.currentTimeMillis()-timeStarted > 500) {
                    robot.drive.setDrivePowers(new PoseVelocity2d(
                            new Vector2d(0, 0), 0
                    ));
                    robot.claw.setPosition(Claw.ClawStates.CLOSE);
                    return false;
                }
            }
            telemetry.update();
            return true;
        };
    }
}