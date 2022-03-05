// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ColorConstants;
import frc.robot.Constants.ConveyorConstants;

public class HorizontalConveyorSubsystem extends SubsystemBase {
  private CANSparkMax m_conveyorSpark = new CANSparkMax(ConveyorConstants.HORIZONTAL_CONVEYOR_SPARK, MotorType.kBrushless);
  private DigitalInput m_beamBreak = new DigitalInput(ConveyorConstants.HORIZONTAL_BEAM_BREAK);

  private ColorSensorV3 m_colorSensor = new ColorSensorV3(I2C.Port.kMXP);
  private final ColorMatch m_colorMatcher = new ColorMatch();

  private SendableChooser<String> teamColor = new SendableChooser<>();

  /** Creates a new ConveyorSubsystem. */
  public HorizontalConveyorSubsystem() {
    m_colorMatcher.setConfidenceThreshold(80);
    m_colorMatcher.addColorMatch(ColorConstants.BLUE_TARGET);
    m_colorMatcher.addColorMatch(ColorConstants.RED_TARGET);
    teamColor.addOption("Red", "red");
    teamColor.addOption("Blue", "blue");
    SmartDashboard.putData(teamColor);
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

  public Color getBallColor() {
    Color color = m_colorMatcher.matchColor(m_colorSensor.getColor()).color;
    if (color == ColorConstants.RED_TARGET) {
      return ColorConstants.RED_TARGET;
    } else if (color == ColorConstants.BLUE_TARGET) {
      return ColorConstants.BLUE_TARGET;
    } else {
      return null;
    }
  }

  public String getTeamColor() {
    return teamColor.getSelected();
  }
}
