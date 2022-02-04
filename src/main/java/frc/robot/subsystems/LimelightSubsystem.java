// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;


public class LimelightSubsystem extends SubsystemBase {
  private final NetworkTable m_limelightTable;

  /** Creates a new LimeLight. */
  public LimelightSubsystem() {
     m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run 
  }

  /**Returns the horizontal offset. If true, returns in encoder units */
  public double getHorizontalOffset(boolean inEncoder) {
    double x = m_limelightTable.getEntry("tx").getDouble(0);
    //this is basically just fancy syntax. Returns tx in encoder units if false
    return inEncoder ? x * 4096/360 : x;
  }

  /**Returns the vertical angle of the limelight away from the target in degrees */
  public double getVerticalOffset() {
    return m_limelightTable.getEntry("ty").getDouble(0);
  }

  /**Returns true if a target is detected */
  public boolean isTargetDetected() {
    return m_limelightTable.getEntry("tv").getDouble(0) == 1;
  }

  // Reference: https://docs.limelightvision.io/en/latest/cs_estimating_distance.html#using-area
  public double getDistance() {
    // Unit: Meters
    double distance = (TurretConstants.HEIGHT_2 - TurretConstants.HEIGHT_1) / Math.toRadians(Math.tan(TurretConstants.ANGLE_1 + getVerticalOffset()));
    return distance;
  }

}
