// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Shoot;
import frc.robot.commands.TurnToApriltag;
import frc.robot.commands.TurnToZero;
import frc.robot.commands.Turnto215;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.StateManager;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.StateManager.States;

public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  // private Command runAuto = drivetrain.getAutoPath("Tests");

  public  final LimelightSubsystem m_limel = new LimelightSubsystem();
  private final Intake   m_Intake   = new Intake();
  private final Wrist    m_Wrist    = new Wrist(m_limel);
  private final Elevator m_Elevator = new Elevator();

  private final StateManager m_StateManager = new StateManager(m_Wrist, m_Elevator, m_Intake, m_limel);
  SendableChooser<Command> autoChooser = new SendableChooser<>();

  private final CommandXboxController m_driverController = new CommandXboxController(Constants.Driver.ControllerPort);
  // private final CommandXboxController m_opController = new CommandXboxController(3);
  private final Joystick m_ButonBoxOne = new Joystick(1);
  private final Joystick m_ButonBoxTwo = new Joystick(2);

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  SendableChooser<Command> chooser = new SendableChooser<>();

  private void configureBindings() {

    m_StateManager.setStates("HOME");
    
    // HOME INTAKE SHOOT AMP HUMANPLAYER MANUAL

    JoystickButton StateHome  = new JoystickButton(m_ButonBoxOne, Constants.ButtonBox1.State_HOME);
    JoystickButton StateIntake  = new JoystickButton(m_ButonBoxOne, Constants.ButtonBox1.State_INTAKE);
    JoystickButton StateShoot  = new JoystickButton(m_ButonBoxOne, Constants.ButtonBox1.State_SHOOT);
    JoystickButton StateAMP  = new JoystickButton(m_ButonBoxOne, Constants.ButtonBox1.State_AMP);
    JoystickButton StateHuman  = new JoystickButton(m_ButonBoxOne, Constants.ButtonBox1.State_HUMANPLAYER);
    JoystickButton StateTrap  = new JoystickButton(m_ButonBoxOne, Constants.ButtonBox1.State_TRAP);
    JoystickButton feed  = new JoystickButton(m_ButonBoxOne, Constants.ButtonBox1.GoLong);
    JoystickButton Outtake  = new JoystickButton(m_ButonBoxOne, Constants.ButtonBox1.Outtake);
    JoystickButton StateClib  = new JoystickButton(m_ButonBoxOne, Constants.ButtonBox1.State_clib);
    JoystickButton GoLong = new JoystickButton(m_ButonBoxOne, Constants.ButtonBox1.GoLong);

    JoystickButton WristDown = new JoystickButton(m_ButonBoxTwo, Constants.ButtonBox2.wristdown);
    JoystickButton WristUp = new JoystickButton(m_ButonBoxTwo, Constants.ButtonBox2.wristup);

    JoystickButton ElevDown = new JoystickButton(m_ButonBoxTwo, Constants.ButtonBox2.elevdown);
    JoystickButton ElevUp = new JoystickButton(m_ButonBoxTwo, Constants.ButtonBox2.elevup);

    JoystickButton Climbdown = new JoystickButton(m_ButonBoxTwo, Constants.ButtonBox2.climbDown);
    JoystickButton FullClimed = new JoystickButton(m_ButonBoxTwo, Constants.ButtonBox2.FullClimed);

    JoystickButton ClimbTrap = new JoystickButton(m_ButonBoxTwo, Constants.ButtonBox2.ClimbTrap);

    JoystickButton DNT = new JoystickButton(m_ButonBoxTwo, Constants.ButtonBox2.DNT);



    StateHome.onTrue( new InstantCommand( () ->  m_StateManager.setStates("HOME")));
    StateIntake.onTrue( new InstantCommand( () ->  m_StateManager.setStates("INTAKE")));
    StateShoot.whileTrue( new InstantCommand( () ->  m_Intake.Feed()));
    StateShoot.onFalse( new InstantCommand( () ->  m_Intake.Stop()));
    StateAMP.onTrue( new InstantCommand( () ->  m_StateManager.setStates("AMP")));
    StateHuman.onTrue( new InstantCommand( () ->  m_StateManager.setStates("HUMANPLAYER")));
    StateIntake.onFalse( new InstantCommand( () ->  m_StateManager.setStates("HOME")));
    StateTrap.onFalse( new InstantCommand( () ->  m_StateManager.setStates("TRAP")));
    StateClib.onTrue( new InstantCommand( () ->  m_StateManager.setStates("CLIMB")));
    GoLong.onTrue( new InstantCommand( () ->  m_StateManager.setStates("GOLONG")));
    ClimbTrap.onTrue(new InstantCommand(()-> m_StateManager.setStates("CLIMBTRAP")));


    WristDown.onTrue(new InstantCommand( () ->  m_Wrist.setAngle(m_Wrist.getAngle()+1)));
    WristUp.onTrue(new InstantCommand( () ->  m_Wrist.setAngle(m_Wrist.getAngle()-3.2)));
    ElevDown.onTrue(new InstantCommand( () ->  m_Elevator.setheight(m_Elevator.getpose()+2)));
    ElevUp.onTrue(new InstantCommand( () ->  m_Elevator.setheight(m_Elevator.getpose()-2)));


    Climbdown.onTrue(new InstantCommand( () ->  m_StateManager.setStates("PULLUP")));
    FullClimed.onTrue(new InstantCommand( () ->  m_StateManager.setStates("CLIMBED")));

    //Gift for naveen dont Touch please
    DNT.whileTrue(new InstantCommand(()-> SmartDashboard.putString("DNT", "Naveen we are super proud of you, we all are very happy you are driveing I hope you have a good season and keep working hard 3/13/24")));
    DNT.whileFalse(new InstantCommand(()-> SmartDashboard.putString("DNT","null")));

    // supposed to rotate to apriltag while b is held, haven't tested yet - preston. 
    m_driverController.b().whileTrue(new  InstantCommand(() ->new TurnToApriltag(drivetrain, m_limel)));
    

    // feed.onTrue(new InstantCommand(() -> m_Intake.Feed()));
    // feed.onFalse(new InstantCommand(() -> m_Intake.Stop()));

    // m_driverController.x().onTrue(new InstantCommand(() -> m_Elevator.RequestAmp()));
    // m_driverController.b().onTrue(new InstantCommand(() -> m_Elevator.RequestHome()));

    // m_driverController.leftBumper().onTrue(new InstantCommand(()-> m_Elevator.Requesttest()));
    // m_driverController.rightBumper().onTrue(new InstantCommand(()-> m_Elevator.Requestclib()));

    
    m_driverController.leftTrigger().whileTrue( new TurnToApriltag(drivetrain, m_limel));
    m_driverController.leftTrigger().whileTrue(new InstantCommand(()-> m_StateManager.setStates("SHOOT")));
    // m_driverController.leftTrigger().whileTrue(new Shoot(m_Elevator, m_Wrist, m_Intake, m_StateManager));
    m_driverController.leftTrigger().onFalse(new InstantCommand(()-> m_StateManager.setStates("HOME")));



    // m_driverController.rightTrigger().onTrue(new Turnto215(drivetrain));
    // m_driverController.rightTrigger().onFalse(new InstantCommand(()-> m_Intake.Stop()));
    // m_driverController.leftTrigger().onFalse(new InstantCommand(()->));
    

    // m_opController.leftBumper().onTrue(new InstantCommand(()->SmartDashboard.putNumber("distance?", m_ll.getdistance())));

  
    // Outtake.onTrue(new SequentialCommandGroup(new InstantCommand(() -> m_Intake.IntakeOut()), new WaitCommand(.4), new InstantCommand(() -> m_Intake.Feed())));
    Outtake.onTrue(new InstantCommand(() -> m_Intake.IntakeOut()));
    Outtake.onFalse(new InstantCommand(() -> m_Intake.Stop()));

    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    // joystick.b().whileTrue(drivetrain
    //     .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press
    joystick.a().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public RobotContainer() {

    Auto.Init(m_StateManager, drivetrain, m_Intake, m_Wrist, m_Elevator, m_limel);


    SmartDashboard.putData("Auto Mode", chooser);

    chooser.addOption("Amp Side 1 + 0 + leave", new PathPlannerAuto("amp1,0"));

    chooser.addOption("Center 1 + 0 + leave", new PathPlannerAuto("cent1,0"));

    chooser.addOption("Source 1 + 0 + Leave" , new PathPlannerAuto("source1,0"));
            
    chooser.addOption("Amp side 1 + 1 leave", new PathPlannerAuto("Amp 1 + 1"));

    chooser.addOption("Center side 1 + 1 leave", new PathPlannerAuto("Center 1 + 1"));

    chooser.addOption("Source side 1 + 1 leave", new PathPlannerAuto("Source 1 + 1"));

    chooser.setDefaultOption("donothing 1", new PathPlannerAuto("Shoot"));

    

    
    /* Auton Chooser */
    // SmartDashboard.putData("Auto Mode", autoChooser);


    //   autoChooser.addOption("Straight Line Test", new PathPlannerAuto("Straight Line Test"));
    //   autoChooser.addOption("Rotation Test", new PathPlannerAuto("Rotation Test"));
    //   autoChooser.addOption("2 Piece", new PathPlannerAuto("2 Piece"));

    configureBindings();

     
    // NamedCommands.registerCommand("Turn to Apriltag", new TurnToApriltag(drivetrain, limelight));

  }

  public Command getAutonomousCommand() {
    // return Commands.print("No autonomous command configured");
    // return null;
    // return new PathPlannerAuto("Center Two Piece");
    // return new PathPlannerAuto("Amp Two Piece");
    // return new PathPlannerAuto("amp2");
    // return new PathPlannerAuto("Shoot Test");
    // return new PathPlannerAuto("cent1,0");

    return chooser.getSelected();
   
    // return autoChooser.getSelected();
  }
}
