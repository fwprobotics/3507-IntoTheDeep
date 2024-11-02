package com.example.meepmeeptesting;


import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;

public class FieldTrajectoryPlanner {

    TrajectoryActionBuilder builder;
    Robot robot;
    public FieldTrajectoryPlanner(Robot robot, Pose2d startingPos) {
        this.builder = robot.drive.getDrive().actionBuilder(startingPos);
        this.robot = robot;
    }

    public FieldTrajectoryPlanner dropSpecimen() {
        builder = builder.strafeToLinearHeading(new Vector2d(5*robot.autoPos.xMult, 40*robot.autoPos.yMult), Math.toRadians(-90*robot.autoPos.yMult))
                .stopAndAdd(new SleepAction(0.75))
                .strafeToLinearHeading(new Vector2d(5*robot.autoPos.xMult, 44*robot.autoPos.yMult), Math.toRadians(-90*robot.autoPos.yMult));

        return this;
    }

    public FieldTrajectoryPlanner pickNeutral(int number) {
        builder = builder.strafeToLinearHeading(new Vector2d((48+(11*number))*robot.autoPos.yMult, 42*robot.autoPos.yMult), Math.toRadians(-90*robot.autoPos.yMult));
        builder = builder.stopAndAdd(new SleepAction(1));
        return this;
    }

    public FieldTrajectoryPlanner dropNet() {
        builder = builder.strafeToLinearHeading(new Vector2d(52*robot.autoPos.yMult, 52*robot.autoPos.yMult), Math.toRadians(robot.autoPos.yMult > 0 ? 45 : 225));
        builder = builder.stopAndAdd(new SleepAction(1));
        return this;
    }

    public FieldTrajectoryPlanner ascend() {
        builder = builder.strafeToLinearHeading(new Vector2d(23*robot.autoPos.yMult, 10*robot.autoPos.yMult),  Math.toRadians(robot.autoPos.yMult > 0 ? 180 : 0));
        return this;
    }

    public FieldTrajectoryPlanner findCoord(Pose2d pose2d) {
        Vector2d bottomCorner = new Vector2d(-24, -24);
        Vector2d upperCorner = new Vector2d(24, 24);

//        Rectangle rect = new Rectangle(upperCorner, bottomCorner);
//        Line line = new Line(new Vector2d(-9, -63), pose2d.position);
//        System.out.println(doesLineIntersectRectangle(line, rect));
        Vector2d currentPose = new Vector2d(-24, -12);

        if (Math.abs(currentPose.x) <= 24) {
            builder = builder.strafeTo(new Vector2d(currentPose.x+(currentPose.x < 0 ? -24 : 24), currentPose.y));
            currentPose = new Vector2d(currentPose.x+(currentPose.x < 0 ? -24 : 24), currentPose.y);
        }

        boolean intersectsXP = lineFuncX(currentPose, pose2d.position, 24);
        boolean intersectsYP = lineFuncY(currentPose, pose2d.position, 24);
        boolean intersectsXM = lineFuncX(currentPose, pose2d.position, -24);
        boolean intersectsYM = lineFuncY(currentPose, pose2d.position, -24);

        while(intersectsXP || intersectsXM || intersectsYM || intersectsYP) {
            System.out.println("kdij");
            if ((intersectsXP && intersectsXM) || ((intersectsYM || intersectsYP) && Math.abs(currentPose.x) >= 24)) {
                System.out.println("kdij");
                builder =  builder.strafeTo(new Vector2d(currentPose.x, 48*(pose2d.position.y > 1 ? 1 : -1)));
                currentPose = new Vector2d(currentPose.x, 48*(pose2d.position.y > 1 ? 1 : -1));
            } else if(intersectsYM || intersectsYP) {
                builder =   builder.strafeTo(new Vector2d(48*(pose2d.position.x > 1 ? 1 : -1), currentPose.y));
                currentPose = new Vector2d(48*(pose2d.position.x > 1 ? 1 : -1), currentPose.y);
//               builder =  builder.strafeTo(new Vector2d(currentPose.x, -60));
//                currentPose = new Vector2d(currentPose.x, -60);
            }
            intersectsXP = lineFuncX(currentPose, pose2d.position, 24);
            intersectsYP = lineFuncY(currentPose, pose2d.position, 24);
          intersectsXM = lineFuncX(currentPose, pose2d.position, -24);
            intersectsYM = lineFuncY(currentPose, pose2d.position, -24);
            System.out.println(intersectsXM + " "+intersectsXP+" "+intersectsYM+" "+intersectsYP);

        }
        builder = builder.strafeTo(pose2d.position);
        return this;
    }

    public boolean lineFuncX(Vector2d start, Vector2d dest, double x) {

        double slope = (dest.y - start.y)/(dest.x- start.x);
        double yValTop =  slope*(x-(dest.x-9))+dest.y+9;
        double yValLow =  slope*(x-(dest.x+9))+dest.y-9;
        boolean topHits = yValTop > -24 && yValTop < 24;
        boolean lowHits = yValTop > -24 && yValTop < 24;
        System.out.println(yValLow+" x"+slope);

        //    System.out.println(yVal+" "+slope);
        return topHits || lowHits || (start.y > -24 && start.y < 24);
    }

    public boolean lineFuncY(Vector2d start, Vector2d dest, double y) {

        double slope = (dest.y - start.y)/(dest.x- start.x);
        double xValTop = (y-dest.y-9)/slope + dest.x-9;
        double xValLow = (y-dest.y+9)/slope + dest.x+9;

        boolean topHits = xValTop > -24 && xValTop < 24;
        boolean lowHits = xValLow > -24 && xValLow < 24;
            System.out.println(xValLow+" "+slope);
        return topHits || lowHits || ((start.x > -24 && start.x < 24));
    }

    public static int orientation(Vector2d p, Vector2d q, Vector2d r) {
        double val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);

        if (val == 0.0) return 0;  // collinear
        return (val > 0) ? 1 : 2;  // clock or counterclock wise
    }

    // Check if point q lies on segment pr
    public static boolean onSegment(Vector2d p, Vector2d q, Vector2d r) {
        return q.x <= Math.max(p.x, r.x) && q.x >= Math.min(p.x, r.x) &&
                q.y <= Math.max(p.y, r.y) && q.y >= Math.min(p.y, r.y);
    }

    // Check if two lines intersect
    public static boolean doIntersect(Line line1, Line line2) {
        Vector2d p1 = line1.start, q1 = line1.end;
        Vector2d p2 = line2.start, q2 = line2.end;

        int o1 = orientation(p1, q1, p2);
        int o2 = orientation(p1, q1, q2);
        int o3 = orientation(p2, q2, p1);
        int o4 = orientation(p2, q2, q1);

        // General case
        if (o1 != o2 && o3 != o4) return true;

        // Special cases
        if (o1 == 0 && onSegment(p1, p2, q1)) return true;
        if (o2 == 0 && onSegment(p1, q2, q1)) return true;
        if (o3 == 0 && onSegment(p2, p1, q2)) return true;
        if (o4 == 0 && onSegment(p2, q1, q2)) return true;

        return false; // No intersection
    }

    public static boolean doesLineIntersectRectangle(Line line, Rectangle rect) {

        if (rect.isLineOutsideBoundingBox(line)) {
            return false;
        }
        Line[] edges = rect.getEdges();

        for (Line edge : edges) {
            if (doIntersect(line, edge)) {
                return true;
            }
        }

        return false;
    }

    class Line {
        Vector2d start, end;

        public Line(Vector2d start, Vector2d end) {
            this.start = start;
            this.end = end;
        }
    }

    class Rectangle {
        Vector2d topLeft, bottomRight;

        public Rectangle(Vector2d topLeft, Vector2d bottomRight) {
            this.topLeft = topLeft;
            this.bottomRight = bottomRight;
        }

        // Get the four edges of the rectangle as lines
        public Line[] getEdges() {
            Vector2d topRight = new Vector2d(bottomRight.x, topLeft.y);
            Vector2d bottomLeft = new Vector2d(topLeft.x, bottomRight.y);

            return new Line[] {
                    new Line(topLeft, topRight),   // top edge
                    new Line(topRight, bottomRight), // right edge
                    new Line(bottomRight, bottomLeft), // bottom edge
                    new Line(bottomLeft, topLeft)  // left edge
            };
        }

        // Check if the line is outside the rectangle's bounding box
        public boolean isLineOutsideBoundingBox(Line line) {
            double minX = Math.min(topLeft.x, bottomRight.x);
            double maxX = Math.max(topLeft.x, bottomRight.x);
            double minY = Math.min(topLeft.y, bottomRight.y);
            double maxY = Math.max(topLeft.y, bottomRight.y);

            // Check if both endpoints of the line are outside the rectangle's bounding box
            if ((line.start.x < minX && line.end.x < minX) || (line.start.x > maxX && line.end.x > maxX) ||
                    (line.start.y < minY && line.end.y < minY) || (line.start.y > maxY && line.end.y > maxY)) {
                return true; // Line is completely outside the rectangle
            }
            return false; // Line may intersect the rectangle
        }
    }
}
