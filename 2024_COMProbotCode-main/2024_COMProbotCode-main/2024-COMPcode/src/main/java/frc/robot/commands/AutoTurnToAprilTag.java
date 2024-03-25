package frc.robot.commands;

import org.opencv.core.Mat;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem;

// Done :)
public class AutoTurnToAprilTag extends Command {
    LimelightSubsystem limelightSubsystem;
    CommandSwerveDrivetrain swerveSubsystem;
    boolean isdone = false;

    public AutoTurnToAprilTag(CommandSwerveDrivetrain swerve, LimelightSubsystem limelight) {
        addRequirements(limelight, swerve);
        limelightSubsystem = limelight;
        swerveSubsystem = swerve;
    } 

    @Override
    public void execute() {
        isdone = false;
        double angle = 0;
        double omega = 0;
        while (!isdone) {
            if (limelightSubsystem.hasTarget()) {
                angle = limelightSubsystem.getApriltagX();
                omega = Math.signum(angle);
                if (Math.abs(angle) < 2) {
                    isdone = true;
                    break;
                }
            }

            swerveSubsystem.driveRobotRelative(new ChassisSpeeds(0, 0, omega * 0.75));
        }
        
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return isdone;
    }

    
}
