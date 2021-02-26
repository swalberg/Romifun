// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.external.RomiGyro;

public class RomiDrivetrain extends SubsystemBase {
  private static final double kCountsPerRevolution = 1440.0;
  private static final double kWheelDiameterM = 70.0/1000.0; // 70 mm
  private DigitalOutput red = new DigitalOutput(2);
  private DigitalOutput green = new DigitalOutput(1);

  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  private final Spark leftMotor = new Spark(0);
  private final Spark rightMotor = new Spark(1);

  // Set up the RomiGyro
  private final RomiGyro gyro = new RomiGyro();
  // Set up the BuiltInAccelerometer
  private final BuiltInAccelerometer accelerometer = new BuiltInAccelerometer();
  private final DifferentialDriveOdometry odometry;

  // The Romi has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder leftEncoder = new Encoder(4, 5);
  private final Encoder rightEncoder = new Encoder(6, 7);

  // Set up the differential drive controller
  private final DifferentialDrive diffDrive = new DifferentialDrive(leftMotor, rightMotor);

  private Timer timer = new Timer();
  /** Creates a new RomiDrivetrain. */
  public RomiDrivetrain() {
    red.set(false);
    green.set(false);
    leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterM) / kCountsPerRevolution);
    rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterM) / kCountsPerRevolution);
    resetEncoders();
    timer.start();
    odometry = new DifferentialDriveOdometry(gyro.getRotation2d());

  }

  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    if (Math.abs(xaxisSpeed) < 0.02) {
      red.set(true);
      green.set(false);
    } else {
      red.set(false);
      green.set(true);
    }
    diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
  }

  public void resetEncoders() {
    gyro.reset();
    leftEncoder.reset();
    rightEncoder.reset();
  }

  public double getLeftDistance() {
    return leftEncoder.getDistance();
  }

  public double getRightDistance() {
    return rightEncoder.getDistance();
  }
  
  public double getHeading() {
    return gyro.getAngleZ();
  }

  @Override
  public void periodic() {
    odometry.update(gyro.getRotation2d(), getLeftDistance(), getRightDistance());
    SmartDashboard.putNumber("x-rate", gyro.getRateX());
    SmartDashboard.putNumber("y-rate", gyro.getRateY());
    SmartDashboard.putNumber("z-rate", gyro.getRateZ());
    SmartDashboard.putNumber("Angle", gyro.getAngleZ());
    SmartDashboard.putString("Pose", getPose().toString());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }
  
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, gyro.getRotation2d());
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMotor.setVoltage(leftVolts);
    rightMotor.setVoltage(-rightVolts);
    diffDrive.feed();
  }
}
