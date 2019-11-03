package frc.robot;

public class Config {
    public double maxVel;
    public double maxAcc;
    public double maxAngVel;
    public double spacing;
    public double lookAheadDistance;
    public double trackWidth;
    public double targetTolerance;
    public double kS;
    public double kV;
    public double kA;

    public static Config getRobotConfig() {
        Config robotConfig = new Config();
        robotConfig.maxVel = Units.ft2m(9);
        robotConfig.maxAcc = Units.ft2m(4);
        robotConfig.spacing = Units.ft2m(0.5);
        robotConfig.maxAngVel = 2;
        robotConfig.lookAheadDistance = Units.ft2m(1.8);
        robotConfig.trackWidth = Units.in2m(25);
        robotConfig.targetTolerance = Units.in2m(2);
        robotConfig.kS = 0;
        robotConfig.kV = 0;
        robotConfig.kA = 0;
        return robotConfig;
    }

    public static Config getPracticeRobotConfig() {
        Config practiceRobotConfig = new Config();
        practiceRobotConfig.maxVel = Units.ft2m(9); // 7 before
        practiceRobotConfig.maxAcc = Units.ft2m(4); // m/sec every sec
        practiceRobotConfig.spacing = Units.ft2m(1);
        practiceRobotConfig.maxAngVel = 2; // radians per second
        practiceRobotConfig.lookAheadDistance = Units.ft2m(1.8);
        practiceRobotConfig.trackWidth = Units.in2m(25);// 23 inches
        practiceRobotConfig.targetTolerance = Units.in2m(2);// m
        practiceRobotConfig.kS = 0.0;
        practiceRobotConfig.kV = 0;
        practiceRobotConfig.kA = 0;
        return practiceRobotConfig;
    }

    @Override
    public String toString() {
        return "Config [kA=" + kA + ", kS=" + kS + ", kV=" + kV + ", lookAheadDistance=" + lookAheadDistance
                + ", maxAcc=" + maxAcc + ", maxAngVel=" + maxAngVel + ", maxVel=" + maxVel + ", spacing=" + spacing
                + ", targetTolerance=" + targetTolerance + ", trackWidth=" + trackWidth + "]";
    }
}