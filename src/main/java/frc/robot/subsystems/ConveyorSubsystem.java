// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConveyorConstants;

public class ConveyorSubsystem extends SubsystemBase {
  private CANSparkMax m_horizontalConveyorSpark = new CANSparkMax(ConveyorConstants.HORIZONTAL_CONVEYOR_SPARK, MotorType.kBrushless);
  private CANSparkMax m_verticalConveyorSpark = new CANSparkMax(ConveyorConstants.VERTICAL_CONVEYOR_SPARK, MotorType.kBrushless);
  private DigitalInput m_horizontalBeamBreak = new DigitalInput(ConveyorConstants.HORIZONTAL_BEAM_BREAK);
  private DigitalInput m_verticalBeamBreak = new DigitalInput(ConveyorConstants.VERTICAL_BEAM_BREAK);

  /** Creates a new ConveyorSubsystem. */
  public ConveyorSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
