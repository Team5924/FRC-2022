// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveDistance extends CommandBase {
  private final DriveSubsystem m_drive;

  private double startingPosition;
  private double inches;
  private double feetPerSecond;

  /** Creates a new DriveDistance. */
  public DriveDistance(DriveSubsystem driveSubsystem, double inches, double feetPerSecond) {
    m_drive = driveSubsystem;
    this.inches = inches;
    this.feetPerSecond = feetPerSecond;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Command drives straight, so only one side needs to be monitored for starting distance
    startingPosition = m_drive.getLeftPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*
    Divide by 10 to change to per 1s from per 100ms (sensor u)
    Multiply by 2048 to get sensor units from rotations
    Divide by 4π to get rotations from inches
    Multiply by 12 to get feet from inches
    Multiply by 9 to account for gearbox
    */
    m_drive.setLeftVelocity(feetPerSecond / 10 * 2048 / (4 * Math.PI) * 12 * 9);
    m_drive.setRightVelocity(feetPerSecond / 10 * 2048 / (4 * Math.PI) * 12 * 9);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.stopLeft();
    m_drive.stopRight();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (inches >= 0) {
      return m_drive.getLeftPosition() >= startingPosition + inches / DriveConstants.WHEEL_CIRCUMFERENCE * 2048 * 9;
    } else {
      return m_drive.getLeftPosition() < startingPosition + inches / DriveConstants.WHEEL_CIRCUMFERENCE * 2048 * 9;
    }
  }
}
