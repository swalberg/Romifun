// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.RomiDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurnToHeading extends PIDCommand {
  private double heading;
  private RomiDrivetrain driveTrain;
  public TurnToHeading(RomiDrivetrain driveTrain, double heading) {
    super(
        // The controller that the command will use
        new PIDController(0.01, 0, 0),
        // This should return the measurement
        () -> driveTrain.getHeading(),
        // This should return the setpoint (can also be a constant)
        () -> heading,
        // This uses the output
        output -> {
          driveTrain.arcadeDrive(0, output);
        });
    this.heading = heading;
    this.driveTrain = driveTrain;
    addRequirements(driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(2);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putBoolean("TurnAround", getController().atSetpoint());
    return getController().atSetpoint();
  }
}
