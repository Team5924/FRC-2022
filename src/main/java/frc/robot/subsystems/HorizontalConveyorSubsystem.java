// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConveyorConstants;

public class HorizontalConveyorSubsystem extends SubsystemBase {
  private CANSparkMax m_conveyorSpark = new CANSparkMax(ConveyorConstants.HORIZONTAL_CONVEYOR_SPARK, MotorType.kBrushless);
  private DigitalInput m_beamBreak = new DigitalInput(ConveyorConstants.HORIZONTAL_BEAM_BREAK);

  private final Color blueTarget = new Color(0.143, 0.427, 0.429);
  private final Color redTarget = new Color(0.561, 0.232, 0.114);
  private final ColorMatch m_colorMatcher = new ColorMatch();

  /** Creates a new ConveyorSubsystem. */
  public HorizontalConveyorSubsystem() {
    m_colorMatcher.setConfidenceThreshold(80);
    m_colorMatcher.addColorMatch(blueTarget);
    m_colorMatcher.addColorMatch(redTarget);
  }

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

  public String getColorFromSensor() {
    double[] rawColor = NetworkTableInstance.getDefault().getEntry("rawcolor1").getDoubleArray(new double[]{0, 0, 0, 0});
    double r = rawColor[0];
    double g = rawColor[1];
    double b = rawColor[2];
    double mag = r + g + b;
    Color detectedColor = new Color(r / mag, g / mag, b / mag);
    ColorMatchResult matchResult = m_colorMatcher.matchClosestColor(detectedColor);
    if (matchResult.color == blueTarget) {
      return "Blue";
    } else if (matchResult.color == redTarget) {
      return "Red";
    } else {
      return "Unknown";
    }
  }
}
