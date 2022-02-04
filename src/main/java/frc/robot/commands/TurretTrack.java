package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class TurretTrack extends CommandBase{
    private TurretSubsystem m_turret;
    private LimelightSubsystem m_limelight;
    private XboxController m_operatorController;

    public TurretTrack(TurretSubsystem subsystem, LimelightSubsystem ls, XboxController controller) {
        m_turret= subsystem;
        m_limelight= ls;
        m_operatorController = controller;
        addRequirements(m_turret, m_limelight);
    }

    @Override
    public void initialize(){
        m_turret.configureTurret(); //Configures the turret to run at max velocity + acceleration
    }

    @Override
    public void execute() {
        double error = m_limelight.getHorizontalOffset(true); //Gets the x angle from the limelight
        SmartDashboard.putNumber("Error", error);
        double currentPOS = m_turret.getPOS();
        error = -error + currentPOS;

        double LeftTrigger = m_operatorController.getLeftTriggerAxis();
        double RightTrigger = m_operatorController.getRightTriggerAxis();

        // Uses trig to get distance | Reference: https://docs.limelightvision.io/en/latest/cs_estimating_distance.html#using-area
        // measured in feet
        double distance = m_limelight.getDistance();

        SmartDashboard.putNumber("POS", currentPOS);
        // Display distance on Shuffleboard
        SmartDashboard.putNumber("Distance", distance);

        if(LeftTrigger > 0.1 && RightTrigger == 0 && error > -TurretConstants.DEGREE) {
            //setting rumble to be off
            m_operatorController.setRumble(RumbleType.kLeftRumble, 0);
            m_operatorController.setRumble(RumbleType.kRightRumble, 0);

            //calling moveleft
            m_turret.moveLeft(LeftTrigger);
        }
        else if ( LeftTrigger == 0 && RightTrigger > 0.1 && error < TurretConstants.DEGREE) {
            //setting rumber to be off
            m_operatorController.setRumble(RumbleType.kLeftRumble, 0);
            m_operatorController.setRumble(RumbleType.kRightRumble, 0);

            //calling moveRight
            m_turret.moveRight(RightTrigger);
        }
        else if(error > -1024 && error < 1024) {
            SmartDashboard.putBoolean("Overconstrained", false);
            m_turret.PIDmove(error);
        }
        else {
            SmartDashboard.putBoolean("Overconstrained", true);
            m_operatorController.setRumble(RumbleType.kLeftRumble, 1.0);
            m_operatorController.setRumble(RumbleType.kRightRumble, 1.0);
            if(currentPOS < TurretConstants.DEGREE) {
                m_turret.PIDmove(-TurretConstants.DEGREE + 10);
            }
            else if (currentPOS > -TurretConstants.DEGREE) {
                m_turret.PIDmove(TurretConstants.DEGREE - 10);
            }
        }
      }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interupted){
    }
}
