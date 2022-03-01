package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

    /*
        The shooter has two motors spinning, relative to each other,
        in the opposite dierction. The leaderSparkMax is also the
        PIDController, and the other one follows.
    */
    private CANSparkMax m_leaderShooterSpark = new CANSparkMax(ShooterConstants.LEADER_SHOOTER_SPARK, MotorType.kBrushless);
    private CANSparkMax m_followerShooterSpark = new CANSparkMax(ShooterConstants.FOLLOWER_SHOOTER_SPARK, MotorType.kBrushless);

    private SparkMaxPIDController m_PIDController;

    private RelativeEncoder m_encoder;

    public ShooterSubsystem() {
        m_leaderShooterSpark.restoreFactoryDefaults();
        m_followerShooterSpark.restoreFactoryDefaults();

        m_followerShooterSpark.follow(m_leaderShooterSpark);

        m_PIDController = m_leaderShooterSpark.getPIDController();

        m_encoder = m_leaderShooterSpark.getEncoder();

        m_PIDController.setP(ShooterConstants.kP);
        m_PIDController.setI(ShooterConstants.kI);
        m_PIDController.setD(ShooterConstants.kD);
        m_PIDController.setFF(ShooterConstants.kF);
    }

    private double distanceToSpeed(double distance) {
        // Write a converstion method for distance to speed
        return distance;
    }

    public void setSpeedForDistance(double distance) {
        double speed = distanceToSpeed(distance);
        m_PIDController.setReference(speed, ControlType.kVelocity);
    }

    public boolean isAtSpeedForDistance(double distance) {
        // Write a converstion method for distance to speed later
        double speed = distanceToSpeed(distance);
        return Math.abs(m_encoder.getVelocity() - speed) <= ShooterConstants.ACCEPTABLE_RPM_ERROR;
    }
}
