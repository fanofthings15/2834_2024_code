package frc.robot;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoTurnToAprilTag;
import frc.robot.commands.TurnTo180;
import frc.robot.commands.TurnToApriltag;
import frc.robot.commands.TurnToZero;
import frc.robot.commands.Turnto215;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.StateManager;
import frc.robot.subsystems.Wrist;

public class Auto {
    public static void Init(StateManager manager, CommandSwerveDrivetrain drivetrain, Intake intake, Wrist wrist, Elevator elevator, LimelightSubsystem limelight) {
        AutoBuilder.configureHolonomic(
            drivetrain::getPose, // Robot pose supplier
            drivetrain::seedFieldRelative, // Method to reset odometry (will be called if your auto has a starting pose)
            drivetrain::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            drivetrain::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            // new PIDConstants(7.9, 0, 0.002), // Translation PID
            //     new PIDConstants( // Rotation PID
            //         .5, 
            //         0, 
            //         0),
                    new PIDConstants(1, 0, 0), // Translation PID constants
                    new PIDConstants(.26, 0, 0), // Rotation PID constants
                    5.5, // Max module swpeed, in m/s
                    .5, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }

              
              return false;
            },
            drivetrain// Reference to this subsystem to set requirements
        );
        
        NamedCommands.registerCommand("Shoot", new SequentialCommandGroup(
            new InstantCommand(() -> {
                manager.setStates("SHOOT");
            }),
            new WaitCommand(2),
            new InstantCommand(() -> intake.Feed()),
            new WaitCommand(0.2),
            new InstantCommand(() -> manager.setStates("HOME"))
        ));

        NamedCommands.registerCommand("Shoot2", new SequentialCommandGroup(
            new InstantCommand(() -> {
                manager.setStates("SHOOT");
            }),
            new WaitCommand(2),
            new InstantCommand(() -> intake.Feed()),
            new WaitCommand(0.2),
            new InstantCommand(() -> manager.setStates("HOME"))
        ));

        

        NamedCommands.registerCommand("Intake", 
        new SequentialCommandGroup(
                new InstantCommand(
                () -> manager.setStates("INTAKE")),
                new WaitCommand(.5))
        );

        NamedCommands.registerCommand("Intake Up", 
            new InstantCommand(() -> manager.setStates("HOME")));
    

        NamedCommands.registerCommand("Wait", new WaitCommand(1));

        NamedCommands.registerCommand("TurnTOZero", new TurnToZero(drivetrain));
        
        NamedCommands.registerCommand("TurnTo180", new TurnTo180(drivetrain));

        NamedCommands.registerCommand("TurnTo215", new Turnto215(drivetrain));

        NamedCommands.registerCommand("Turn TO april tag", new AutoTurnToAprilTag(drivetrain, limelight));

        NamedCommands.registerCommand("Wait.25", new WaitCommand(.25));

        
        
    }

}
