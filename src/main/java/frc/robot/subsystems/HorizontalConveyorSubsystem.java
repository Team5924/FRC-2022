// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConveyorConstants;

public class HorizontalConveyorSubsystem extends SubsystemBase {
  private CANSparkMax m_conveyorSpark = new CANSparkMax(ConveyorConstants.HORIZONTAL_CONVEYOR_SPARK, MotorType.kBrushless);
  private RelativeEncoder m_encoder;
  private DigitalInput m_beamBreak = new DigitalInput(ConveyorConstants.HORIZONTAL_BEAM_BREAK);

  private final Color blueBallTarget = new Color(0.152, 0.421, 0.426);
  private final Color redBallTarget = new Color(0.564, 0.327, 0.112);
  private final ColorMatch m_colorMatcher = new ColorMatch();

  private boolean lastBallSameColor = false;

  /** Creates a new ConveyorSubsystem. */
  public HorizontalConveyorSubsystem() {
    m_encoder = m_conveyorSpark.getEncoder();

    m_colorMatcher.addColorMatch(blueBallTarget);
    m_colorMatcher.addColorMatch(redBallTarget);
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("Color Detected", getColorFromSensor());
    SmartDashboard.putBoolean("Horizontal Beam", isBeamBroken());
    SmartDashboard.putBoolean("Is Red Alliance", isRedAlliance());
    SmartDashboard.putBoolean("Last Ball Same Color", isLastBallSameColor());

    if (getColorFromSensor().equals("red")) {
      if (isRedAlliance()) {
        lastBallSameColor = true;
      } else {
        lastBallSameColor = false;
      }
    } else if (getColorFromSensor().equals("blue")) {
      if (isRedAlliance()) {
        lastBallSameColor = false;
      } else {
        lastBallSameColor = true;
      }
    }
  }

  public void enableConveyor() {
    m_conveyorSpark.set(0.25);
  }

  public void disableConveyor() {
    m_conveyorSpark.stopMotor();
  }

  public void poopBall() {
    m_conveyorSpark.set(0.6);
  }

  public boolean isConveyorRunning() {
    return Math.abs(m_encoder.getVelocity()) > 1;
  }

  public boolean isBeamBroken() {
    return !m_beamBreak.get();
  }

  private static boolean isRedAlliance() {
    return NetworkTableInstance.getDefault().getTable("FMSInfo").getEntry("IsRedAlliance").getBoolean(false);
  }

  private String getColorFromSensor() {
    double[] rawColor = NetworkTableInstance.getDefault().getEntry("/rawcolor1").getDoubleArray(new double[]{0, 0, 0, 0});
    double r = rawColor[0];
    double g = rawColor[1];
    double b = rawColor[2];
    double mag = r + g + b;
    SmartDashboard.putNumber("R", r / mag);
    SmartDashboard.putNumber("G", g / mag);
    SmartDashboard.putNumber("B", b / mag);
    Color detectedColor = new Color(r / mag, g / mag, b / mag);
    ColorMatchResult matchResult = m_colorMatcher.matchColor(detectedColor);
    if (matchResult == null) {
      return "none";
    } else if (matchResult.color.equals(blueBallTarget)) {
      return "blue";
    } else if (matchResult.color.equals(redBallTarget)) {
      return "red";
    } else {
      return "none";
    }
  }

  public boolean isLastBallSameColor() {
    return lastBallSameColor;
  }
}
