// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.auto.AutoShoot;
import frc.robot.commands.auto.Taxi;
import frc.robot.commands.MaintainShooterSpeed;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VerticalConveyorSubsystem;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SingleBallAuto extends SequentialCommandGroup {
  private ShooterSubsystem m_shooter;
  private DriveSubsystem m_drivetrain;
  private VerticalConveyorSubsystem m_verticalConveyor;

  /** Creates a new AutoAimAndShoot. */
  public SingleBallAuto(ShooterSubsystem shooterSubsystem, DriveSubsystem driveSubsystem, VerticalConveyorSubsystem verticalConveyorSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    m_shooter = shooterSubsystem;
    m_verticalConveyor = verticalConveyorSubsystem;
    m_drivetrain = driveSubsystem;

    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoShoot(m_verticalConveyor, m_shooter),
      new Taxi(m_drivetrain)
    );
  }
}
