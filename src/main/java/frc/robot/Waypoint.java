package frc.robot;

public class Waypoint extends Point{
    public double v = 0;

    public Waypoint(double x, double y, double v) {
        super(x, y);
        this.v = v;
    }

    public Waypoint(double x, double y) {
        this(x, y, 0);
    }

    public String toString() {
        return x + ", " + y + ", " + v;
    }
}