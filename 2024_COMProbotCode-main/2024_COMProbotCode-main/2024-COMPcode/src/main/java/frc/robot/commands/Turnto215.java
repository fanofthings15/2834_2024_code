package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;

// Done :)
public class Turnto215 extends Command {
    CommandSwerveDrivetrain m_drive;
    double rotation;

    public Turnto215(CommandSwerveDrivetrain swerve) {
        m_drive = swerve;
    } 

    @Override
    public void execute() {
        rotation = m_drive.getPigeon2().getAngle();

        if(DriverStation.getAlliance().orElse(Alliance.Red)==Alliance.Blue)
            rotation -= 212;
        else{
            rotation += 212;
        }

        

        SmartDashboard.putNumber("rotation of bot", rotation);


        while (rotation > 180) {
            rotation -= 360;
        }
        while (rotation < -180) {
            rotation += 360;
        }



        double run = Math.signum(rotation);
        SmartDashboard.putNumber("Robot Angle", rotation);

        m_drive.driveRobotRelative(new ChassisSpeeds(0,0,-run*1.3));

    }

    @Override
    public void end(boolean interrupted) {
       m_drive.driveRobotRelative(new ChassisSpeeds(0,0,0));
    }

    @Override
    public boolean isFinished() {

        return Math.abs(rotation) < .5;
    }
}
