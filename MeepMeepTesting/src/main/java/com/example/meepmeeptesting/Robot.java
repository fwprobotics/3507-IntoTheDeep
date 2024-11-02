package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

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

    public RoadRunnerBotEntity drive;
    public AutoPos autoPos;

    public Robot(RoadRunnerBotEntity drive, AutoPos pos) {
        this.drive = drive;
        this.autoPos = pos;
    }

    public FieldTrajectoryPlanner createTrajectoryPlanner() {
        Pose2d startingPos = new Pose2d(24*autoPos.xMult, 12* autoPos.yMult, Math.toRadians(0* autoPos.yMult));
        return new FieldTrajectoryPlanner(this, startingPos);
    }
}
