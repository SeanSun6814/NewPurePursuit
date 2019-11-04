package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;

import frc.robot.Waypoint;

public class PathGenerator {

    double numFinalPoints;

    public static void main(String[] args) {
        PathGenerator pathGenerator = new PathGenerator();
        Config config = Config.getPracticeRobotConfig();

        ArrayList<Waypoint> waypoints = new ArrayList<Waypoint>(
                Arrays.asList(new Waypoint(1.7235169999999997, 4.092194), new Waypoint(2.4474169999999997, 4.092194),
                        new Waypoint(4.092702, 3.8159690000000004), new Waypoint(4.803902, 3.8159690000000004)));

        waypoints = pathGenerator.generate(waypoints, config);
        System.out.println("Path: [" + waypoints.size() + "]");
        for (int i = 0; i < waypoints.size(); i++) {
            System.out.println(i + "]: " + waypoints.get(i));
        }
        System.out.println("__________________________\n\n");
    }

    public ArrayList<Waypoint> generate(ArrayList<Waypoint> nodeOnlyPath, Config config) {
        int[] inject = injectionCounter2Steps(nodeOnlyPath, config);

        ArrayList<Waypoint> smoothPath = new ArrayList<Waypoint>();
        smoothPath = inject(nodeOnlyPath, inject[0]);
        smoothPath = smoother(smoothPath, 0.7, 0.3, 0.0000001);
        // iteratively inject and smooth the path
        for (int i = 1; i < inject.length; i++) {
            smoothPath = inject(smoothPath, inject[i]);
            smoothPath = smoother(smoothPath, 0.1, 0.3, 0.0000001);
        }
        tagVelocity(smoothPath, config);

        return smoothPath;
    }

    public int[] injectionCounter2Steps(ArrayList<Waypoint> path, Config config) {
        int first = 0;
        int second = 0;
        int third = 0;

        double oldPointsTotal = 0;
        numFinalPoints = 0;

        int[] ret = null;
        double totalPoints = getPathDistance(path) / config.spacing;
        totalPoints = Math.max(totalPoints, 51);

        int numNodeOnlyPoints = path.size();
        if (totalPoints < 100) {
            double pointsFirst = 0;
            double pointsTotal = 0;
            for (int i = 4; i <= 6; i++) {
                for (int j = 1; j <= 8; j++) {
                    pointsFirst = i * (numNodeOnlyPoints - 1) + numNodeOnlyPoints;
                    pointsTotal = (j * (pointsFirst - 1) + pointsFirst);
                    if (pointsTotal <= totalPoints && pointsTotal > oldPointsTotal) {
                        first = i;
                        second = j;
                        numFinalPoints = pointsTotal;
                        oldPointsTotal = pointsTotal;
                    }
                }
            }
            ret = new int[] { first, second, third };
        } else {
            double pointsFirst = 0;
            double pointsSecond = 0;
            double pointsTotal = 0;
            for (int i = 1; i <= 5; i++)
                for (int j = 1; j <= 8; j++)
                    for (int k = 1; k < 8; k++) {
                        pointsFirst = i * (numNodeOnlyPoints - 1) + numNodeOnlyPoints;
                        pointsSecond = (j * (pointsFirst - 1) + pointsFirst);
                        pointsTotal = (k * (pointsSecond - 1) + pointsSecond);

                        if (pointsTotal <= totalPoints) {
                            first = i;
                            second = j;
                            third = k;
                            numFinalPoints = pointsTotal;
                        }
                    }

            ret = new int[] { first, second, third };
        }
        return ret;
    }

    public ArrayList<Waypoint> inject(ArrayList<Waypoint> path, int numToInject) {
        double[][] orig = waypoints2Array(path);
        double morePoints[][];
        morePoints = new double[orig.length + ((numToInject) * (orig.length - 1))][2];
        int index = 0;

        for (int i = 0; i < orig.length - 1; i++) {
            morePoints[index][0] = orig[i][0];
            morePoints[index][1] = orig[i][1];
            index++;
            for (int j = 1; j < numToInject + 1; j++) {
                morePoints[index][0] = j * ((orig[i + 1][0] - orig[i][0]) / (numToInject + 1)) + orig[i][0];
                morePoints[index][1] = j * ((orig[i + 1][1] - orig[i][1]) / (numToInject + 1)) + orig[i][1];
                index++;
            }
        }
        morePoints[index][0] = orig[orig.length - 1][0];
        morePoints[index][1] = orig[orig.length - 1][1];
        return array2Waypoints(morePoints);
    }

    public ArrayList<Waypoint> smoother(ArrayList<Waypoint> OrigPath, double weight_data, double weight_smooth,
            double tolerance) {

        double[][] path = waypoints2Array(OrigPath);
        // copy array
        double[][] newPath = waypoints2Array(OrigPath);

        double change = tolerance;
        while (change >= tolerance) {
            change = 0.0;
            for (int i = 1; i < path.length - 1; i++)
                for (int j = 0; j < path[i].length; j++) {
                    double aux = newPath[i][j];
                    newPath[i][j] += weight_data * (path[i][j] - newPath[i][j])
                            + weight_smooth * (newPath[i - 1][j] + newPath[i + 1][j] - (2.0 * newPath[i][j]));
                    change += Math.abs(aux - newPath[i][j]);
                }
        }

        return array2Waypoints(newPath);
    }

    public void tagVelocity(ArrayList<Waypoint> path, Config config) {

        ArrayList<Waypoint> waypoints = path;

        for (int i = 0; i < waypoints.size(); i++) {
            double curvature = 0;
            if (i != 0 && i != waypoints.size() - 1) {
                double x1 = waypoints.get(i).x + 0.0001, y1 = waypoints.get(i).y + 0.0001;
                double x2 = waypoints.get(i - 1).x, y2 = waypoints.get(i - 1).y;
                double x3 = waypoints.get(i + 1).x, y3 = waypoints.get(i + 1).y;

                double k1 = 0.5 * (x1 * x1 + y1 * y1 - x2 * x2 - y2 * y2) / (x1 - x2);
                double k2 = (y1 - y2) / (x1 - x2);
                double b = 0.5 * (x2 * x2 - 2 * x2 * k1 + y2 * y2 - x3 * x3 + 2 * x3 * k1 - y3 * y3)
                        / (x3 * k2 - y3 + y2 - x2 * k2);
                double a = k1 - k2 * b;
                double r = Math.sqrt((x1 - a) * (x1 - a) + (y1 - b) * (y1 - b));
                curvature = 1 / r;
            }
            waypoints.get(i).v = Math.min(config.maxVel, config.maxAngVel / curvature);
        }

        waypoints.get(waypoints.size() - 1).v = 0;

        for (int i = waypoints.size() - 1 - 1; i >= 0; i--) {
            double distance = distanceBetween(waypoints.get(i + 1), waypoints.get(i));

            waypoints.get(i).v = Math.min(waypoints.get(i).v,
                    getMaxAchieveableVelocity(config.maxAcc, distance, waypoints.get(i + 1).v));
        }
    }

    private double getMaxAchieveableVelocity(double timeAccel, double distance, double velocity0) {
        return Math.sqrt(velocity0 * velocity0 + 2 * timeAccel * distance);
    }

    public double[][] waypoints2Array(ArrayList<Waypoint> waypoints) {
        double[][] arr = new double[waypoints.size()][2];
        for (int i = 0; i < waypoints.size(); i++) {
            arr[i][0] = waypoints.get(i).x;
            arr[i][1] = waypoints.get(i).y;
        }
        return arr;
    }

    public ArrayList<Waypoint> array2Waypoints(double[][] arr) {
        ArrayList<Waypoint> waypoints = new ArrayList<Waypoint>();
        for (int i = 0; i < arr.length; i++) {
            waypoints.add(new Waypoint(arr[i][0], arr[i][1]));
        }
        return waypoints;
    }

    private double distanceBetween(Point a, Point b) {
        return Math.sqrt(Math.pow(b.x - a.x, 2) + Math.pow(b.y - a.y, 2));
    }

    public double getPathDistance(ArrayList<Waypoint> path) {
        double distance = 0;
        for (int i = 0; i < path.size() - 1; i++) {
            distance += distanceBetween(path.get(i), path.get(i + 1));
        }
        return distance;
    }

}