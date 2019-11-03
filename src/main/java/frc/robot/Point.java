package frc.robot;

public class Point {
    public double x = 0;
    public double y = 0;

    public Point(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Point subtract(Point p) {
        return new Point(this.x - p.x, this.y - p.y);
    }

    public String toString() {
        return x + ", " + y;
    }
}