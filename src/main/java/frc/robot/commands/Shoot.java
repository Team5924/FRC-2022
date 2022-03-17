// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VerticalConveyorSubsystem;

public class Shoot extends CommandBase {
  private final VerticalConveyorSubsystem m_verticalConveyor;
  //private final LimelightSubsystem m_limelight;
  private final ShooterSubsystem m_shooter; 

  /** Creates a new Shoot. */
  public Shoot(VerticalConveyorSubsystem verticalConveyorSubsystem, /*LimelightSubsystem limelightSubsystem,*/ ShooterSubsystem shooterSubsystem) {
    m_verticalConveyor = verticalConveyorSubsystem;
    //m_limelight = limelightSubsystem;
    m_shooter = shooterSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(verticalConveyorSubsystem, shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_verticalConveyor.feedBallToShooter();
    // Runs the shooter at specific speed based on the distance from the target
    //m_shooter.setSpeed(m_shooter.shotVelocityToShooterRPM(m_shooter.getShotVelocity(m_limelight.getDistance())));
    //if (isAtSpeed) {
      // The vertical conveyor feeds a ball into the shooter, when shooter is ready to fire
      //m_verticalConveyor.enableConveyor();
    /*} else if (m_shooter.isShooterAtSpeed()) {
      isAtSpeed = true;
    }*/
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_verticalConveyor.disableConveyor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
