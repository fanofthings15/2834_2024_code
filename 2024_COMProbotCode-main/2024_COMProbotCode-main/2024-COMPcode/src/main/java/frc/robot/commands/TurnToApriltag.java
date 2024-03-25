package frc.robot.commands;

import org.opencv.core.Mat;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem;

// Done :)
public class TurnToApriltag extends Command {
    LimelightSubsystem limelightSubsystem;
    CommandSwerveDrivetrain swerveSubsystem;

    public TurnToApriltag(CommandSwerveDrivetrain swerve, LimelightSubsystem limelight) {
        addRequirements(limelight, swerve);
        limelightSubsystem = limelight;
        swerveSubsystem = swerve;
    } 

    @Override
    public void execute() {
        if(!(Math.abs(limelightSubsystem.getApriltagX()) < 1)) {
            double omega = Math.signum(limelightSubsystem.getApriltagX());
            SmartDashboard.putNumber("Limelight Omega", omega);
            swerveSubsystem.driveRobotRelative(new ChassisSpeeds(0, 0, omega));
        }

        end(false);
    }
}
