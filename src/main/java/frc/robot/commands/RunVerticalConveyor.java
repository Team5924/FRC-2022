// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HorizontalConveyorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VerticalConveyorSubsystem;

public class RunVerticalConveyor extends CommandBase {
  private final VerticalConveyorSubsystem m_verticalConveyor;
  private final HorizontalConveyorSubsystem m_horizontalConveyor;
  private final IntakeSubsystem m_intake;

  private boolean isDisablingConveyor = false;
  private int conveyorDisableDelay = 150;
  private long disableConveyorAt = 0;

  /** Creates a new RunVerticalConveyor. */
  public RunVerticalConveyor(HorizontalConveyorSubsystem horizontalConveyorSubsystem, VerticalConveyorSubsystem verticalConveyorSubsystem, IntakeSubsystem intakeSubsystem) {
    m_verticalConveyor = verticalConveyorSubsystem;
    m_horizontalConveyor = horizontalConveyorSubsystem;
    m_intake = intakeSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_verticalConveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //if (m_intake.isIntakeMotorRunning() && m_intake.isIntakeDeployed()) {
      if (m_verticalConveyor.isBeamBroken()) {
        if (isDisablingConveyor) {
          if (System.currentTimeMillis() >= disableConveyorAt) {
            m_verticalConveyor.disableConveyor();
          }
        } else {
          if (m_verticalConveyor.isConveyorRunning()) {
            isDisablingConveyor = true;
            disableConveyorAt = System.currentTimeMillis() + conveyorDisableDelay;
          }
        }
      } else if (!m_horizontalConveyor.isLastBallSameColor()) {
        m_verticalConveyor.disableConveyor();
      } else {
        m_verticalConveyor.enableConveyor();
      }
    //} else {
    //  m_verticalConveyor.disableConveyor();
    //}
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
