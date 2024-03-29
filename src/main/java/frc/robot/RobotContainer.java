// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveManualTrajectory;
import frc.robot.commands.TurnToHeading;
import frc.robot.subsystems.RomiDrivetrain;
import frc.robot.subsystems.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final RomiDrivetrain m_romiDrivetrain = new RomiDrivetrain();
  private final Vision vision = new Vision();

  // Commands
  private final TurnToHeading turnToBack = new TurnToHeading(m_romiDrivetrain, 180);

  // Inputs
  private final Joystick joystick = new Joystick(0);
  private final JoystickButton buttonA = new JoystickButton(joystick, Button.kA.value);
  private final JoystickButton buttonB = new JoystickButton(joystick, Button.kB.value);

  // Auto commands
  private SendableChooser<Command> m_chooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    populateAutonomousCommands();
    m_romiDrivetrain.resetEncoders();
    m_romiDrivetrain.setDefaultCommand(new RunCommand(() -> m_romiDrivetrain.arcadeDrive(-joystick.getY(), joystick.getX()), m_romiDrivetrain));
    m_romiDrivetrain.resetEncoders();
    SmartDashboard.putData(m_romiDrivetrain);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    buttonA.whileHeld(() -> vision.inc());
    buttonB.whileHeld(() -> vision.dec());
  }

  private void populateAutonomousCommands() {
    m_chooser.setDefaultOption("S-Curve", new DriveManualTrajectory(m_romiDrivetrain).makeCommand());
    m_chooser.addOption("Turn around", turnToBack);
    SmartDashboard.putData(m_chooser);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    m_romiDrivetrain.resetEncoders();
    return m_chooser.getSelected();
  }

  public void robotPeriodic() {
  }
}