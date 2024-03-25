package frc.robot.commands;

import org.opencv.core.Mat;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.StateManager;
import frc.robot.subsystems.Wrist;

// Done :)
public class Shoot extends Command {
    Elevator m_elev;
    Wrist m_Wrist;
    Intake m_Intake;
    StateManager m_man;

    public Shoot(Elevator elev, Wrist wrist, Intake intake, StateManager man) {
        m_elev = elev;
        m_Wrist = wrist;
        m_Intake = intake;
        m_man = man;
    } 

    @Override
    public void execute() {
        m_man.setStates("SHOOT");

        end(false);
    }
}
