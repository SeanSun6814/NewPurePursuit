/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private Joystick driverJoystick = new Joystick(0);

  private TalonSRX leftMaster = new TalonSRX(3);
  private TalonSRX rightMaster = new TalonSRX(1);
  private VictorSPX leftSlave = new VictorSPX(1);
  private VictorSPX rightSlave = new VictorSPX(2);
  private final double kDriveTick2Feet = 1.0 / 4096 * 6 * Math.PI / 12;

  public AHRS gyro;

  // ------Controllers------------
  private double prevTimestamp = 0;
  private Odometer odometer = new Odometer();
  private PathGenerator pathGenerator = new PathGenerator();
  private PathFollower pathFollower;
  private Logger logger = new Logger();
  private Config config;
  private boolean driveReversed = false;

  @Override
  public void robotInit() {
    leftMaster.setInverted(true);
    rightMaster.setInverted(false);
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);
    leftSlave.setInverted(InvertType.FollowMaster);
    rightSlave.setInverted(InvertType.FollowMaster);

    leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

    leftMaster.setSensorPhase(false);
    rightMaster.setSensorPhase(false);

    ArrayList<Waypoint> path = new ArrayList<Waypoint>(
        Arrays.asList(new Waypoint(1.7235169999999997, 4.092194), new Waypoint(2.4474169999999997, 4.092194),
            new Waypoint(4.092702, 3.8159690000000004), new Waypoint(4.803902, 3.8159690000000004)));

    config = Config.getRobotConfig();
    path = pathGenerator.generate(path, config);
    pathFollower = new PathFollower(path, config);

    logger.log("Config", config);
    logger.log("TotalDistance", pathFollower.pathTotalDistance);

    path.forEach((waypoint) -> {
      logger.log("Path", waypoint);
    });

  }

  @Override
  public void robotPeriodic() {

  }

  @Override
  public void autonomousInit() {
    resetSensors();
    odometer.set(pathFollower.path.get(0));
    prevLeftSpeed = 0;
    prevRightSpeed = 0;
    prevTimestamp = Timer.getFPGATimestamp();
  }

  double dt = 0;
  double prevLeftSpeed = 0;
  double prevRightSpeed = 0;

  double leftSpeed;
  double rightSpeed;
  double leftAccel;
  double rightAccel;
  double leftOutput;
  double rightOutput;

  @Override
  public void autonomousPeriodic() {
    dt = Timer.getFPGATimestamp() - prevTimestamp;
    odometer.update(getLeftEncoder(), getRightEncoder(), getGyroAngle());

    double pathFollowerGyro = getGyroAngle();
    if (driveReversed)
      pathFollowerGyro *= -1;

    MotorOutputs speeds = pathFollower.update(odometer.get(), pathFollowerGyro, dt);
    leftSpeed = speeds.left;
    rightSpeed = speeds.right;

    leftAccel = (leftSpeed - prevLeftSpeed) * (1 / dt);
    rightAccel = (rightSpeed - prevRightSpeed) * (1 / dt);

    leftOutput = leftSpeed * config.kV + leftAccel * config.kA;
    rightOutput = rightSpeed * config.kV + rightAccel * config.kA;

    if (leftSpeed > 0.001) {
      leftOutput += config.kS;
    } else if (leftSpeed < -0.001) {
      leftOutput -= config.kS;
    }

    if (rightSpeed > 0.001) {
      rightOutput += config.kS;
    } else if (rightSpeed < -0.001) {
      rightOutput -= config.kS;
    }

    if (driveReversed) {
      leftOutput *= -1;
      rightOutput *= -1;
    }
    setMotors(leftOutput, rightOutput);

    logData();

    prevLeftSpeed = leftSpeed;
    prevRightSpeed = rightSpeed;
  }

  @Override
  public void disabledInit() {
    logger.flush();
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    double speed = -driverJoystick.getRawAxis(1) * 0.6;
    double turn = driverJoystick.getRawAxis(4) * 0.3;
    setMotors(speed + turn, speed - turn);
  }

  public void setMotors(double left, double right) {
    leftMaster.set(ControlMode.PercentOutput, left);
    rightMaster.set(ControlMode.PercentOutput, right);
  }

  public double getLeftEncoder() {
    return leftMaster.getSelectedSensorPosition(0) * kDriveTick2Feet;
  }

  public double getRightEncoder() {
    return rightMaster.getSelectedSensorPosition(0) * kDriveTick2Feet;
  }

  public double getGyroAngle() {
    return gyro.getAngle();
  }

  public void resetSensors() {
    leftMaster.setSelectedSensorPosition(0, 0, 10);
    rightMaster.setSelectedSensorPosition(0, 0, 10);
    gyro.reset();
  }

  public void logData() {
    logger.log("angle", pathFollower.angle);
    logger.log("angle", pathFollower.dt);
    logger.log("robotPos", pathFollower.robotPos);
    logger.log("onPath", pathFollower.onPath);
    logger.log("isDone", pathFollower.isDone);
    logger.log("progress", pathFollower.progress);
    logger.log("backClosestWaypointIndex", pathFollower.backClosestWaypointIndex);
    logger.log("frontClosestWaypointIndex", pathFollower.frontClosestWaypointIndex);
    logger.log("robotInterpolatedPos", pathFollower.robotInterpolatedPos);
    logger.log("lookAheadPoint", pathFollower.lookAheadPoint);
    logger.log("curvature", pathFollower.curvature);
    logger.log("dist2Target", pathFollower.dist2Target);
    logger.log("targetVelocity", pathFollower.targetVelocity);

    logger.log("DrivingDirection", driveReversed);
    logger.log("leftSpeed", leftSpeed);
    logger.log("rightSpeed", rightSpeed);
    logger.log("leftAccel", leftAccel);
    logger.log("rightAccel", rightAccel);
    logger.log("leftOutput", leftOutput);
    logger.log("rightOutput", rightOutput);
    logger.log("leftActualSpeed", leftMaster.getSelectedSensorVelocity() * kDriveTick2Feet * 10);
    logger.log("rightActualSpeed", rightMaster.getSelectedSensorVelocity() * kDriveTick2Feet * 10);
  }

  public void displaySmartDashboard() {
    SmartDashboard.putNumber("Timestamp", Timer.getFPGATimestamp());
    SmartDashboard.putNumber("dt", dt);
    SmartDashboard.putNumber("x position", odometer.x);
    SmartDashboard.putNumber("y position", odometer.y);
  }
}
