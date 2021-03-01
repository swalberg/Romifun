// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.RomiDrivetrain;

public class DriveManualTrajectory {
  /** Creates a new DriveManualTrajectory. */
  RomiDrivetrain m_romiDrivetrain;
  public DriveManualTrajectory(RomiDrivetrain m_romiDrivetrain) {
    this.m_romiDrivetrain = m_romiDrivetrain;
  }

  public Command makeCommand() {
    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
            DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics, 6);

    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)), List.of(new Translation2d(1.0, 0.0)
        // new Translation2d(2.0, 1.0),
        // new Translation2d(2.0, -1.0)
        ),
        // new Pose2d(0, 0, new Rotation2d(Math.PI)),
        new Pose2d(1.5, 0.6, new Rotation2d(0)), config);

    RamseteController disabledRamsete = new RamseteController() {
      @Override
      public ChassisSpeeds calculate(Pose2d currentPose, Pose2d poseRef, double linearVelocityRefMeters,
          double angularVelocityRefRadiansPerSecond) {
        return new ChassisSpeeds(linearVelocityRefMeters, 0.0, angularVelocityRefRadiansPerSecond);
      }
    };
    var table = NetworkTableInstance.getDefault().getTable("troubleshooting");
    var leftReference = table.getEntry("left_reference");
    var leftMeasurement = table.getEntry("left_measurement");
    var rightReference = table.getEntry("right_reference");
    var rightMeasurement = table.getEntry("right_measurement");
    var leftController = new PIDController(DriveConstants.kPDriveVel, 0, 0);
    var rightController = new PIDController(DriveConstants.kPDriveVel, 0, 0);

    RamseteCommand ramseteCommand = new RamseteCommand(exampleTrajectory, m_romiDrivetrain::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        // disabledRamsete,
        new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
            DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics, m_romiDrivetrain::getWheelSpeeds, leftController, rightController,
        // RamseteCommand passes volts to the callback
        (leftVolts, rightVolts) -> {
          m_romiDrivetrain.tankDriveVolts(leftVolts, rightVolts);

          leftMeasurement.setNumber(m_romiDrivetrain.getWheelSpeeds().leftMetersPerSecond);
          leftReference.setNumber(leftController.getSetpoint());

          rightMeasurement.setNumber(m_romiDrivetrain.getWheelSpeeds().rightMetersPerSecond);
          rightReference.setNumber(rightController.getSetpoint());
        }, m_romiDrivetrain);

    // Reset odometry to the starting pose of the trajectory.
    m_romiDrivetrain.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return new InstantCommand(() -> m_romiDrivetrain.resetEncoders()).andThen(ramseteCommand)
        .andThen(() -> m_romiDrivetrain.tankDriveVolts(0, 0));

  }
}
