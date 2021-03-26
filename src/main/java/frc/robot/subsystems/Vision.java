// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  private final int bounds = 20; // degrees
  private double angle;
  Servo rotator = new Servo(2);
  /** Creates a new Vision. */
  public Vision() {
    angle = 0;
  }
  public void dec() {
    angle--;
    setAngle(angle);
  }

  public void inc() {
    angle++;
    setAngle(angle);
  }

  private void setAngle(double angle) {
    rotator.setAngle(angle);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Vision angle", angle);
  }
}
