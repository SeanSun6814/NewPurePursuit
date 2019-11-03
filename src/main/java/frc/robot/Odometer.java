package frc.robot;

public class Odometer {

    public double x;
    public double y;

    private double prevLeftEncoderValue;
    private double prevRightEncoderValue;

    public void update(double leftEncoder, double rightEncoder, double gyroAngle) {
        gyroAngle = Math.toRadians(gyroAngle + 90);

        gyroAngle = Math.PI - gyroAngle;

        double deltaLeftEncoder = leftEncoder - prevLeftEncoderValue;
        double deltaRightEncoder = rightEncoder - prevRightEncoderValue;
        double distance = (deltaLeftEncoder + deltaRightEncoder) / 2;

        x += distance * Math.cos(gyroAngle);
        y += distance * Math.sin(gyroAngle);

        prevLeftEncoderValue = leftEncoder;
        prevRightEncoderValue = rightEncoder;
    }

    public void reset() {
        x = y = prevLeftEncoderValue = prevRightEncoderValue = 0;
    }

    public Point get() {
        return new Point(x, y);
    }
}