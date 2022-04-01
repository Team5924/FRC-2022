package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

    private final Compressor m_compressor = new Compressor(IntakeConstants.CTRE_PCM, PneumaticsModuleType.CTREPCM);

    private final Solenoid m_intakeSolenoid = new Solenoid(IntakeConstants.CTRE_PCM, PneumaticsModuleType.CTREPCM, IntakeConstants.INTAKE_PNEUMATICS);

    private final CANSparkMax m_intakeSpark = new CANSparkMax(IntakeConstants.INTAKE_SPARK, MotorType.kBrushless);

    public IntakeSubsystem() {
        //m_compressor.disable();

        m_intakeSolenoid.set(false);

        m_intakeSpark.setInverted(true);
        m_intakeSpark.restoreFactoryDefaults();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Pneumatic Extended?", m_intakeSolenoid.get());
        SmartDashboard.putBoolean("Compressor Status", m_compressor.enabled());
        SmartDashboard.putBoolean("Intake Deployed", isDeployed());
        SmartDashboard.putBoolean("Compressor Pressurized", m_compressor.getPressureSwitchValue());
    }

    public void enableCompressor() {
        m_compressor.enableDigital();
    }

    public void disableCompressor() {
        m_compressor.disable();
    }

    public boolean isCompressorOn() {
        return m_compressor.enabled();
    }

    public void deploy() {
        m_intakeSolenoid.set(true);
    }

    public void retract() {
        m_intakeSolenoid.set(false);
    }

    public boolean isDeployed() {
        // returns the status of the intake, used to determine whether to retract or deploy intake
        return m_intakeSolenoid.get();
    }

    public void setIntakeMotorForward() {
        m_intakeSpark.set(IntakeConstants.INTAKE_RUN_SPEED);
    }

    public void stopIntakeMotor() {
        m_intakeSpark.stopMotor();
    }
}
