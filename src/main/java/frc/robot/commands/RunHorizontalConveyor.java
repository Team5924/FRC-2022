// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HorizontalConveyorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VerticalConveyorSubsystem;

public class RunHorizontalConveyor extends CommandBase {
  private final HorizontalConveyorSubsystem m_horizontalConveyor;
  private final VerticalConveyorSubsystem m_verticalConveyor;
  private final IntakeSubsystem m_intake;

  private boolean lastBallSameColor = false;

  /** Creates a new RunConveyor. */
  public RunHorizontalConveyor(HorizontalConveyorSubsystem horizontalConveyorSubsystem, VerticalConveyorSubsystem verticalConveyorSubsystem, IntakeSubsystem intakeSubsystem) {
    m_horizontalConveyor = horizontalConveyorSubsystem;
    m_verticalConveyor = verticalConveyorSubsystem;
    m_intake = intakeSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_horizontalConveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_horizontalConveyor.getColorFromSensor().equals("Red")) {
      if (isRedAlliance()) {
        lastBallSameColor = true;
      } else {
        lastBallSameColor = false;
      }
    } else if (m_horizontalConveyor.getColorFromSensor().equals("Blue")) {
      if (isRedAlliance()) {
        lastBallSameColor = false;
      } else {
        lastBallSameColor = true;
      }
    }

    if (m_intake.isIntakeMotorRunning() && m_intake.isIntakeDeployed()) {
      if (!m_horizontalConveyor.isBeamBroken() && !m_verticalConveyor.isBeamBroken() && lastBallSameColor) {
        m_horizontalConveyor.disableConveyor();
      } else {
        m_horizontalConveyor.enableConveyor();
      }
    } else {
      m_horizontalConveyor.disableConveyor();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private static boolean isRedAlliance() {
    return NetworkTableInstance.getDefault().getTable("FMSInfo").getEntry("IsRedAlliance").getBoolean(false);
  }
}
