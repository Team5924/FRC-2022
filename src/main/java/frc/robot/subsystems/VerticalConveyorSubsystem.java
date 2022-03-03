// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConveyorConstants;

public class VerticalConveyorSubsystem extends SubsystemBase {
  private DigitalInput m_beamBreak = new DigitalInput(ConveyorConstants.HORIZONTAL_BEAM_BREAK);
  private CANSparkMax m_conveyorSpark = new CANSparkMax(ConveyorConstants.VERTICAL_CONVEYOR_SPARK, MotorType.kBrushless);

  /** Creates a new VerticalConveyorSubsystem. */
  public VerticalConveyorSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void enableConveyor() {
    m_conveyorSpark.set(0.3);
  }

  public void disableConveyor() {
    m_conveyorSpark.stopMotor();
  }

  public boolean isBeamCompleted() {
    return m_beamBreak.get();
  }
}
