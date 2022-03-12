// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.TargetHub;
import frc.robot.commands.auto.AutoShoot;
import frc.robot.commands.auto.AutoTaxi;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VerticalConveyorSubsystem;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoAimShootMove extends SequentialCommandGroup {
  private LimelightSubsystem m_limelight;
  private ShooterSubsystem m_shooter;
  private TurretSubsystem m_turret;
  private VerticalConveyorSubsystem m_verticalConveyor;
  private DriveSubsystem m_drive;

  /** Creates a new AutoAimAndShoot. */
  public AutoAimShootMove(LimelightSubsystem limelightSubsystem, ShooterSubsystem shooterSubsystem, TurretSubsystem turretSubsystem, VerticalConveyorSubsystem verticalConveyorSubsystem, DriveSubsystem driveSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    m_limelight = limelightSubsystem;
    m_shooter = shooterSubsystem;
    m_turret = turretSubsystem;
    m_verticalConveyor = verticalConveyorSubsystem;
    m_drive = driveSubsystem;

    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new TargetHub(m_limelight, m_turret),
      new AutoShoot(m_verticalConveyor, m_limelight, m_shooter),
      new AutoTaxi(m_drive)
    );
  }
}
