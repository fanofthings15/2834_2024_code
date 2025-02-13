// package frc.robot;

// import java.util.function.Supplier;

// import com.ctre.phoenix6.Utils;
// import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
// import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
// import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
// import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
// import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
// import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.RobotCentric;
// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
// import com.pathplanner.lib.util.PIDConstants;
// import com.pathplanner.lib.util.ReplanningConfig;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj.Notifier;
// import edu.wpi.first.wpilibj.RobotController;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Subsystem;
// import frc.robot.generated.TunerConstants;

// /**
//  * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
//  * so it can be used in command-based projects easily.
//  */
// public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
//     private static final double kSimLoopPeriod = 0.005; // 5 ms
//     private Notifier m_simNotifier = null;
//     private double m_lastSimTime;

//     public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
//         super(driveTrainConstants, OdometryUpdateFrequency, modules);
//         if (Utils.isSimulation()) {
//             startSimThread();
//         }
//     }
//     public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
//         super(driveTrainConstants, modules);
//         if (Utils.isSimulation()) {
//             startSimThread();
//         }
//     }

//     public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
//         return run(() -> this.setControl(requestSupplier.get()));
//     }

//     private void startSimThread() {
//         m_lastSimTime = Utils.getCurrentTimeSeconds();

//         /* Run simulation at a faster rate so PID gains behave more reasonably */
//         m_simNotifier = new Notifier(() -> {
//             final double currentTime = Utils.getCurrentTimeSeconds();
//             double deltaTime = currentTime - m_lastSimTime;
//             m_lastSimTime = currentTime;

//             /* use the measured time delta, get battery voltage from WPILib */
//             updateSimState(deltaTime, RobotController.getBatteryVoltage());
//         });
//         m_simNotifier.startPeriodic(kSimLoopPeriod);
//     }

//      public Pose2d getPose() {
//         return this.getState().Pose;
//     }

//     public ChassisSpeeds getChassisSpeeds() {
//         return m_kinematics.toChassisSpeeds(getState().ModuleStates);
//     }

//     private void configurePathPlanner() {
//         double driveBaseRadius = 0;
//         for (var moduleLocation : m_moduleLocations) {
//             driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
//         }

//         AutoBuilder.configureHolonomic(
//             ()->this.getState().Pose, // Supplier of current robot pose
//             this::seedFieldRelative,  // Consumer for seeding pose against auto
//             this::getCurrentRobotChassisSpeeds,
//             (speeds)->this.setControl(AutoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the robot
//             new HolonomicPathFollowerConfig(new PIDConstants(10, 0, 0),
//                                             new PIDConstants(10, 0, 0),
//                                             TunerConstants.kSpeedAt12VoltsMps,
//                                             driveBaseRadius,
//                                             new ReplanningConfig()),
//             () -> DriverStation.getAlliance().orElse(Alliance.Blue)==Alliance.Red, // Assume the path needs to be flipped for Red vs Blue, this is normally the case
//             this); // Subsystem for requirements
//     }

//     public ChassisSpeeds getCurrentRobotChassisSpeeds() {
//         return m_kinematics.toChassisSpeeds(getState().ModuleStates);
//     }


//     public void driveRobotRelative(ChassisSpeeds speeds) {
//         RobotCentric driveRequest = new RobotCentric()
//             .withRotationalRate(speeds.omegaRadiansPerSecond)
//             .withVelocityX(speeds.vxMetersPerSecond)
//             .withVelocityY(speeds.vyMetersPerSecond)
//             .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
//         driveRequest.apply(m_requestParameters, Modules);
//     }
// }
