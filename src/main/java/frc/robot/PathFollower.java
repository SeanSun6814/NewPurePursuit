package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;

public class PathFollower {

    public Config config;
    public ArrayList<Waypoint> path;
    public double pathTotalDistance;
    public boolean onPath, isDone;
    public double progress;

    public Point prevLookAheadPoint;
    public int prevBackClosestWaypointIndex;
    public int prevFrontClosestWaypointIndex;
    public double solidDistanceTravelled;
    public double prevTargetVel;
    public double prevDist2Target;

    public int backClosestWaypointIndex;
    public int frontClosestWaypointIndex;
    public Waypoint robotInterpolatedPos;
    public Point lookAheadPoint;
    public double curvature;
    public double dist2Target;
    public double targetVelocity;

    public double angle, dt;
    public Point robotPos;

    public static void main(String[] args) {
        ArrayList<Waypoint> path = new ArrayList<Waypoint>(
                Arrays.asList(new Waypoint(1.7235169999999997, 4.092194), new Waypoint(2.4474169999999997, 4.092194),
                        new Waypoint(4.092702, 3.8159690000000004), new Waypoint(4.803902, 3.8159690000000004)));
        path = new ArrayList<Waypoint>(
                Arrays.asList(new Waypoint(1, 4), new Waypoint(400, 4)));

        Config config = Config.getRobotConfig();
        path = new PathGenerator().generate(path, config);
        PathFollower pathFollower = new PathFollower(path, config);
        System.out.println(pathFollower.pathTotalDistance);
    }

    public PathFollower(ArrayList<Waypoint> path, Config config) {
        this.config = config;
        this.path = path;
        prevLookAheadPoint = null;
        prevBackClosestWaypointIndex = 0;
        prevFrontClosestWaypointIndex = 0;
        solidDistanceTravelled = 0;
        isDone = false;
        onPath = false;
        progress = 0;
        prevTargetVel = 0;
        prevDist2Target = Double.MAX_VALUE;
        getPathTotalDistance();
    }

    public MotorOutputs update(Point robotPos, double angle, double dt) {
        this.robotPos = robotPos;
        this.angle = Math.toRadians(angle + 90);
        this.dt = dt;

        getFinishedPath();
        if (isDone)
            return new MotorOutputs(0, 0);

        getClosestWaypoint();
        getInterpolatedWaypoint();
        getProgress();
        getLookAheadPoint();
        getCurvature();

        if (onPath) {
            targetVelocity = Math.min(robotInterpolatedPos.v, config.maxAngVel / Math.abs(curvature));
        } else {
            targetVelocity = config.maxAngVel / Math.abs(curvature);
        }

        double maxDeltaTargetVelocity = config.maxAcc * dt;

        targetVelocity = clamp(targetVelocity, prevTargetVel - maxDeltaTargetVelocity,
                prevTargetVel + maxDeltaTargetVelocity);

        double leftVel = targetVelocity * (2 + curvature * config.trackWidth) / 2;
        double rightVel = targetVelocity * (2 - curvature * config.trackWidth) / 2;

        updatePrevs();

        return new MotorOutputs(leftVel, rightVel);
    }

    private void getInterpolatedWaypoint() {
        double d1 = distanceBetween(robotPos, path.get(frontClosestWaypointIndex));
        double d2 = distanceBetween(robotPos, path.get(backClosestWaypointIndex));

        Vector deltaVector = new Vector(path.get(frontClosestWaypointIndex), path.get(backClosestWaypointIndex));

        double d = deltaVector.length();

        double scale = (d * d + d1 * d1 - d2 * d2) / (2 * d);

        Vector xVector = deltaVector.scale(scale / d);

        double ratio = xVector.length() / d;

        double vel = path.get(frontClosestWaypointIndex).v
                + ratio * (path.get(frontClosestWaypointIndex).v - path.get(backClosestWaypointIndex).v);

        robotInterpolatedPos = new Waypoint(path.get(frontClosestWaypointIndex).x + xVector.dx,
                path.get(frontClosestWaypointIndex).y + xVector.dy, vel);
    }

    private void getClosestWaypoint() {
        int searchFrom = prevBackClosestWaypointIndex;
        int searchTo = Math.min(searchFrom + 5, path.size());

        int minIndex = searchFrom;
        double minDistance = Double.MAX_VALUE;

        for (int i = searchFrom; i <= searchTo; i++) {
            double distance = distanceBetween(path.get(i), robotPos);
            if (distance < minDistance) {
                minDistance = distance;
                minIndex = i;
            }
        }
        int secondClosestWaypointIndex;
        if (minIndex == 0)
            secondClosestWaypointIndex = minIndex + 1;
        else if (minIndex == path.size() - 1)
            secondClosestWaypointIndex = minIndex - 1;

        if ((distanceBetween(robotPos, path.get(minIndex - 1))) < (distanceBetween(robotPos, path.get(minIndex + 1)))) {
            secondClosestWaypointIndex = minIndex - 1;
        }
        secondClosestWaypointIndex = minIndex + 1;

        frontClosestWaypointIndex = Math.max(minIndex, secondClosestWaypointIndex);
        backClosestWaypointIndex = Math.min(minIndex, secondClosestWaypointIndex);
    }

    private void getLookAheadPoint() {
        int searchFrom = Math.max(backClosestWaypointIndex, 0);// Math.max(getClosestWaypointIndex(robotPos), 0);
        int searchTo = path.size() - 1;

        for (int i = searchFrom; i < searchTo; i++) {
            Point startOfLine = path.get(i);
            Point endOfLine = path.get(i + 1);
            double radius = config.lookAheadDistance;
            Vector d = new Vector(endOfLine.subtract(startOfLine));
            Vector f = new Vector(startOfLine.subtract(robotPos));

            double a = d.dotProduct(d);
            double b = 2 * f.dotProduct(d);
            double c = f.dotProduct(f) - radius * radius;
            double discriminant = Math.sqrt(b * b - 4 * a * c);

            double t1 = (-b + discriminant) / (2 * a);
            double t2 = (-b - discriminant) / (2 * a);

            if (t1 >= 0 && t1 <= 1) {
                Vector v = d.scale(t2);
                lookAheadPoint = new Point(startOfLine.x + v.dx, startOfLine.y + v.dy);
                return;

            }
            if (t2 >= 0 && t2 <= 1) {
                Vector v = d.scale(t1);
                lookAheadPoint = new Point(startOfLine.x + v.dx, startOfLine.y + v.dy);
                return;
            }
        }
        if (prevLookAheadPoint == null) {
            lookAheadPoint = (Point) path.get(frontClosestWaypointIndex);
        }
    }

    public void getCurvature() {

        double gyro = Math.PI - angle;

        double horizontalDistance2LookAheadPoint;
        double a = -Math.tan(gyro);
        double b = 1;
        double c = Math.tan(gyro) * robotPos.x - robotPos.y;

        horizontalDistance2LookAheadPoint = Math.abs(a * lookAheadPoint.x + b * lookAheadPoint.y + c)
                / (Math.sqrt(a * a + b * b));

        double side = Math.signum(
                Math.sin(gyro) * (lookAheadPoint.x - robotPos.x) - Math.cos(gyro) * (lookAheadPoint.y - robotPos.y));

        curvature = 2 * side * horizontalDistance2LookAheadPoint / config.lookAheadDistance / config.lookAheadDistance;
    }

    private void getProgress() {
        if (isDone) {
            progress = 100;
            return;
        }
        int addFrom = Math.min(prevBackClosestWaypointIndex, backClosestWaypointIndex);
        int addTo = Math.max(prevBackClosestWaypointIndex, backClosestWaypointIndex);
        int incDirection = (int) Math.signum(backClosestWaypointIndex - prevBackClosestWaypointIndex);
        for (int i = addFrom; i < addTo; i++) {
            solidDistanceTravelled += distanceBetween(path.get(i), path.get(i + 1)) * incDirection;
        }

        double currentDistance = solidDistanceTravelled
                + distanceBetween(path.get(backClosestWaypointIndex), robotInterpolatedPos);

        progress = currentDistance / pathTotalDistance * 100;
    }

    private void getFinishedPath() {
        dist2Target = distanceBetween(robotPos, path.get(path.size() - 1));
        if (dist2Target <= config.targetTolerance) {
            prevDist2Target = dist2Target;
        }
        if (dist2Target > prevDist2Target) {
            isDone = true;
        }
    }

    private void updatePrevs() {
        prevLookAheadPoint = lookAheadPoint;
        prevBackClosestWaypointIndex = backClosestWaypointIndex;
        prevFrontClosestWaypointIndex = frontClosestWaypointIndex;
        prevTargetVel = targetVelocity;
    }

    private double distanceBetween(Point a, Point b) {
        return Math.sqrt(Math.pow(b.x - a.x, 2) + Math.pow(b.y - a.y, 2));
    }

    private void getPathTotalDistance() {
        for (int i = 0; i < path.size() - 1; i++) {
            pathTotalDistance += distanceBetween(path.get(i), path.get(i + 1));
        }
    }

    private double clamp(double x, double min, double max) {
        if (x < min)
            x = min;
        else if (x > max)
            x = max;
        return x;
    }

}

// class MotorOutputs {
// public double leftVel;
// public double rightVel;

// public MotorOutputs(double leftVel, double rightVel) {
// this.leftVel = leftVel;
// this.rightVel = rightVel;
// }

// // public Vector subtract(Vector v) {
// // Vector v2 = new Vector(this.dx - v.dx, this.dy - v.dy);
// // return v2;
// // }

// // public Vector scale(double scaleFactor) {
// // return new Vector(this.dx * scaleFactor, this.dy * scaleFactor);
// // }

// }
