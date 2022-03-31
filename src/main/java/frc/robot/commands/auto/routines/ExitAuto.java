// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.routines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.ToggleShooter;
import frc.robot.commands.auto.DriveDistance;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ExitAuto extends SequentialCommandGroup {
  private final ShooterSubsystem m_shooter;
  private final DriveSubsystem m_drive;

  /** Creates a new ExitAuto. */
  public ExitAuto(ShooterSubsystem shooterSubsystem, DriveSubsystem driveSubsystem) {
    m_shooter = shooterSubsystem;
    m_drive = driveSubsystem;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ToggleShooter(m_shooter),
      new DriveDistance(m_drive, 40, DriveConstants.AUTO_DRIVE_SPEED)
    );
  }
}
