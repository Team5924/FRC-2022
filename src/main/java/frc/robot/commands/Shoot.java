// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class Shoot extends CommandBase {
  private final ConveyorSubsystem m_conveyor;
  private final LimelightSubsystem m_limelight;
  private final ShooterSubsystem m_shooter;

  /** Creates a new Shoot. */
  public Shoot(ConveyorSubsystem conveyorSubsystem, LimelightSubsystem limelightSubsystem, ShooterSubsystem shooterSubsystem) {
    m_conveyor = conveyorSubsystem;
    m_limelight = limelightSubsystem;
    m_shooter = shooterSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(conveyorSubsystem, shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    
    // m_shooter.setSpeedForDistance(m_limelight.getDistance());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


  /*
    if (m_shooter.isAtSpeedForDistance(m_limelight.getDistance())) {
      m_conveyor.runVerticalConveyor();
    }
  */
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
