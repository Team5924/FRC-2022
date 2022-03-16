// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.MaintainShooterSpeed;
import frc.robot.commands.ReverseIntakeMotor;
import frc.robot.commands.RunHorizontalConveyor;
import frc.robot.commands.RunVerticalConveyor;
import frc.robot.commands.Shoot;
import frc.robot.commands.TankDrive;
import frc.robot.commands.TestIntake;
import frc.robot.commands.ToggleCompressor;
import frc.robot.commands.ToggleIntake;
import frc.robot.commands.auto.SingleBallAuto;
import frc.robot.commands.auto.DoubleBallAuto;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HorizontalConveyorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VerticalConveyorSubsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
  public static final LimelightSubsystem m_limelight = new LimelightSubsystem();
  public static final IntakeSubsystem m_intake = new IntakeSubsystem();
  public static final HorizontalConveyorSubsystem m_horizontalConveyor = new HorizontalConveyorSubsystem();
  public static final VerticalConveyorSubsystem m_verticalConveyor = new VerticalConveyorSubsystem();
  public static final ShooterSubsystem m_shooter = new ShooterSubsystem();
  public static final TurretSubsystem m_turret = new TurretSubsystem();

  XboxController m_driverController = new XboxController(OIConstants.DRIVER_CONTROLLER);

  JoystickButton a = new JoystickButton(m_driverController, XboxController.Button.kA.value);
  JoystickButton b = new JoystickButton(m_driverController, XboxController.Button.kB.value);
  JoystickButton x = new JoystickButton(m_driverController, XboxController.Button.kX.value);
  JoystickButton y = new JoystickButton(m_driverController, XboxController.Button.kY.value);
  JoystickButton rightBumper = new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value);

  // Declaring sendableObject for Autonomous here
  private final Command m_singleAuto = new SingleBallAuto(m_limelight, m_shooter, m_turret, m_verticalConveyor,
      m_drivetrain);
  private final Command m_doubleAuto = new DoubleBallAuto(m_limelight, m_shooter, m_turret, m_verticalConveyor,
      m_drivetrain, m_intake);

  SendableChooser<Command> m_autoChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_intake.register();
    m_horizontalConveyor.register();
    m_verticalConveyor.register();
    m_shooter.register();
    m_turret.register();

    //m_drivetrain.setDefaultCommand(new TankDrive(m_drivetrain, m_driverController::getLeftY, m_driverController::getRightY));
    //m_horizontalConveyor.setDefaultCommand(new RunHorizontalConveyor(m_horizontalConveyor, m_verticalConveyor, m_intake));
    //m_verticalConveyor.setDefaultCommand(new RunVerticalConveyor(m_horizontalConveyor, m_verticalConveyor, m_intake));
    m_shooter.setDefaultCommand(new MaintainShooterSpeed(m_shooter));

    // Configure the button bindings
    configureButtonBindings();

    m_autoChooser.setDefaultOption("Single Ball Auto", m_singleAuto);
    m_autoChooser.addOption("Double Ball Auto", m_doubleAuto);

    SmartDashboard.putData(m_autoChooser);
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
    //a.whenPressed(new ToggleIntake(m_intake));
    //b.whenPressed(new ToggleCompressor(m_intake));
    //y.whenHeld(new ReverseIntakeMotor(m_intake));
    //rightBumper.whenHeld(new Shoot(m_verticalConveyor, m_limelight, m_shooter));
    a.whenHeld(new TestIntake(m_intake));
    b.whenHeld(new Shoot(m_verticalConveyor, m_shooter));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoChooser.getSelected();
  }
}