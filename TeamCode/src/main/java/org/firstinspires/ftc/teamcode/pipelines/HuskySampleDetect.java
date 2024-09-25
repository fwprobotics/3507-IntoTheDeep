package org.firstinspires.ftc.teamcode.pipelines;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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

    public HuskySampleDetect(HardwareMap hardwareMap, Telemetry telemetry) {
        this.huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens");
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
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
} // public Sample getNearestSample() {
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
