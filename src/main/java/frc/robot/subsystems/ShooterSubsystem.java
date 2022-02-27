package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;

import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

    /*
        The shooter has two motors spinning, relative to each other,
        in the opposite dierction. The leaderSparkMax is also the
        PIDController, and the other one follows.
    */
    private CANSparkMax m_masterSpark = new CANSparkMax(ShooterConstants.MASTER_SPARK, MotorType.kBrushless);
    private CANSparkMax m_slaveSpark = new CANSparkMax(ShooterConstants.SLAVE_MASTER, MotorType.kBrushless);
    
    private SparkMaxPIDController m_PIDController;

    public ShooterSubsystem() {

        m_masterSpark.restoreFactoryDefaults();
        m_slaveSpark.restoreFactoryDefaults();

        m_slaveSpark.follow(m_masterSpark); 
        
        m_PIDController = m_masterSpark.getPIDController();

        m_PIDController.setP(ShooterConstants.kP);
        m_PIDController.setI(ShooterConstants.kI);
        m_PIDController.setD(ShooterConstants.kD);
        m_PIDController.setFF(ShooterConstants.kF);
    }

    public void setSpeed(double speed) {
        m_masterSpark.set(speed);
    }

}
