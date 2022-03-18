// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.TankDrive;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.commands.ExtendClimber;
import frc.robot.commands.RetractClimber;
import frc.robot.commands.ToggleShooter;
import frc.robot.commands.RunHorizontalConveyor;
import frc.robot.commands.RunVerticalConveyor;
import frc.robot.commands.Shoot;
import frc.robot.commands.ToggleIntake;
import frc.robot.commands.ToggleShooter;
import frc.robot.commands.auto.SingleBallAuto;
import frc.robot.subsystems.HorizontalConveyorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VerticalConveyorSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static final DriveSubsystem m_drivetrain = new DriveSubsystem();
  public static final IntakeSubsystem m_intake = new IntakeSubsystem();
  public static final HorizontalConveyorSubsystem m_horizontalConveyor = new HorizontalConveyorSubsystem();
  public static final VerticalConveyorSubsystem m_verticalConveyor = new VerticalConveyorSubsystem();
  public static final ShooterSubsystem m_shooter = new ShooterSubsystem();
  public static final ClimberSubsystem m_climber = new ClimberSubsystem();

  XboxController m_driverController = new XboxController(OIConstants.DRIVER_CONTROLLER);

  XboxController m_operatorController = new XboxController(OIConstants.OPERATOR_CONTROLLER);

  JoystickButton driverX = new JoystickButton(m_driverController, XboxController.Button.kX.value);

  JoystickButton operatorB = new JoystickButton(m_operatorController, XboxController.Button.kB.value);
  JoystickButton operatorY = new JoystickButton(m_operatorController, XboxController.Button.kY.value);
  JoystickButton operatorLeftBumper = new JoystickButton(m_operatorController, XboxController.Button.kLeftBumper.value);
  JoystickButton operatorRightBumper = new JoystickButton(m_operatorController, XboxController.Button.kRightBumper.value);

  // Declaring sendableObject for Autonomous here
  private final Command m_singleAuto = new SingleBallAuto(m_shooter, m_drivetrain, m_verticalConveyor);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_intake.register();
    m_horizontalConveyor.register();
    m_verticalConveyor.register();
    m_shooter.register();
    m_drivetrain.register();
    m_climber.register();

    m_drivetrain.setDefaultCommand(new TankDrive(m_drivetrain, m_driverController::getLeftY, m_driverController::getRightY));
    m_horizontalConveyor.setDefaultCommand(new RunHorizontalConveyor(m_horizontalConveyor, m_intake));
    m_verticalConveyor.setDefaultCommand(new RunVerticalConveyor(m_horizontalConveyor, m_verticalConveyor, m_intake));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    driverX.whenPressed(new ToggleIntake(m_intake));

    operatorB.whenHeld(new Shoot(m_verticalConveyor, m_shooter));
    operatorY.whenPressed(new ToggleShooter(m_shooter));
    operatorLeftBumper.whenHeld(new RetractClimber(m_climber));
    operatorRightBumper.whenHeld(new ExtendClimber(m_climber));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_singleAuto;
  }
}