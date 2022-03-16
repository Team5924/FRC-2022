// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConveyorConstants;

public class VerticalConveyorSubsystem extends SubsystemBase {
  private CANSparkMax m_conveyorSpark = new CANSparkMax(ConveyorConstants.VERTICAL_CONVEYOR_SPARK, MotorType.kBrushless);
  private RelativeEncoder m_encoder;
  private DigitalInput m_beamBreak = new DigitalInput(ConveyorConstants.VERTICAL_BEAM_BREAK);

  /** Creates a new VerticalConveyorSubsystem. */
  public VerticalConveyorSubsystem() {
    m_encoder = m_conveyorSpark.getEncoder();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Vertical Beam", isBeamBroken());
    SmartDashboard.putBoolean("Vertical Conveyor Running", isConveyorRunning());
  }

  public void enableConveyor() {
    m_conveyorSpark.set(0.2);
  }

  public void disableConveyor() {
    m_conveyorSpark.stopMotor();
  }

  public void feedBallToShooter() {
    m_conveyorSpark.set(0.5);
  }

  public boolean isConveyorRunning() {
    return Math.abs(m_encoder.getVelocity()) > 1;
  }

  public boolean isBeamBroken() {
    return !m_beamBreak.get();
  }
}
