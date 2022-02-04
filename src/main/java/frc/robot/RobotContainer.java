// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.TankDrive;
import frc.robot.commands.TurretFindLeft;
import frc.robot.commands.TurretFindRight;
import frc.robot.commands.TurretTrack;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static final DriveSubsystem m_drivetrain = new DriveSubsystem();
  public static final LimelightSubsystem m_limelight = new LimelightSubsystem();
  public static final TurretSubsystem m_turret = new TurretSubsystem();

  XboxController m_driverController = new XboxController(OIConstants.DRIVER_CONTROLLER_PORT);
  XboxController m_operatorController = new XboxController(OIConstants.OPERATOR_CONTROLLER_PORT);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    m_drivetrain.setDefaultCommand(new TankDrive(m_drivetrain, m_driverController::getLeftY, m_driverController::getRightY));
    configureButtonBindings();
    m_turret.setDefaultCommand(new TurretTrack(m_turret, m_limelight, m_driverController));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //Creates a new Joystick Button object using the xbox controller buttons. Binds FindLeft and FindRight to them.
    JoystickButton leftBump = new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value);
    leftBump.whenPressed(new TurretFindLeft(m_turret, m_limelight));
    JoystickButton rightBump = new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value);
    rightBump.whenPressed(new TurretFindRight(m_turret, m_limelight));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
