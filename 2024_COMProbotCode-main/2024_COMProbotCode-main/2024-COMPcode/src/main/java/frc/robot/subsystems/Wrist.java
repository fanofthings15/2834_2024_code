// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Wrist extends SubsystemBase {
  TalonFX m_left;
  TalonFX m_right;
  ProfiledPIDController m_pid;
  Constraints trappro;
  MotionMagicVoltage m_motmag;
  LimelightSubsystem m_limelight;

  final double ZERO_OFFSET = 2.2;
  
  public Wrist(LimelightSubsystem limelight) {
    SmartDashboard.putNumber("adushootangle", 0);

    m_limelight = limelight;

    m_right = new TalonFX(Constants.WristConstants.RIGHTID , "Canivore");
    m_left = new TalonFX(Constants.WristConstants.LEFTID , "Canivore");


    m_motmag = new MotionMagicVoltage(0);

    var talonFXConfigs = new TalonFXConfiguration();
    
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = 0.24; // add 0.24 V to overcome friction
    slot0Configs.kV = 0.12; // apply 12 V for a target velocity of 100 rps
    // PID runs on position
    slot0Configs.kP = 1; //1
    slot0Configs.kI = 0;
    slot0Configs.kD = 0;

    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 30; // 80 rps cruise velocity
    motionMagicConfigs.MotionMagicAcceleration = 100; // 160 rps/s acceleration (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 300; // 1600 rps/s^2 jerk (0.1 seconds)

    m_left.getConfigurator().apply(talonFXConfigs, 0.050);
    m_right.getConfigurator().apply(talonFXConfigs, 0.050);
    // periodic, run Motion Magic with slot 0 configs,
  }

  

  @Override
  public void periodic() {
    m_motmag.Slot = 0;
    SmartDashboard.putNumber("Wrist Angle", getAngle());
  }

public Double getAngle(){
  return ((Math.abs(m_left.getPosition().getValue()) + Math.abs(m_right.getPosition().getValue())) / 2) - ZERO_OFFSET;
}
  

  public void RequestHome(){
    m_left.setControl(m_motmag.withPosition(ZERO_OFFSET));
    m_right.setControl(m_motmag.withPosition(-ZERO_OFFSET));
  }

  public void RequestIntake(){
    m_left.setControl(m_motmag.withPosition(12.3 + ZERO_OFFSET));
    m_right.setControl(m_motmag.withPosition(-12.3 - ZERO_OFFSET));
  }

  public void RequestShoot(){
    double distance = m_limelight.getdistance();

    if (distance > 1) {

      double degrees = getanglefromdis(distance);
      
      
      m_left.setControl(m_motmag.withPosition(degrees/10 + ZERO_OFFSET)); //need to find out how the auto target with work
      m_right.setControl(m_motmag.withPosition(-degrees/10 - ZERO_OFFSET));
    } else {
    m_left.setControl(m_motmag.withPosition(2 + ZERO_OFFSET)); //need to find out how the auto target with work
    m_right.setControl(m_motmag.withPosition(-2 - ZERO_OFFSET));//3.5
    }

    SmartDashboard.putNumber("LL DIstance", distance);
    // m_left.setControl(m_motmag.withPosition(SmartDashboard.getNumber("adushootangle", 0)+ZERO_OFFSET)); //need to find out how the auto target with work
    // m_right.setControl(m_motmag.withPosition(-SmartDashboard.getNumber("adushootangle", 0)-ZERO_OFFSET));
    
  }

  public void AutoShoot(){
    m_left.setControl(m_motmag.withPosition(1.8 + ZERO_OFFSET)); //need to find out how the auto target with work
    m_right.setControl(m_motmag.withPosition(-1.8 - ZERO_OFFSET));//3.5
  }
  
  public void RequestHuman(){
    m_left.setControl(m_motmag.withPosition(2.7 + ZERO_OFFSET));
    m_right.setControl(m_motmag.withPosition(-2.7 - ZERO_OFFSET));
  }

  public void RequestAmp(){
    m_left.setControl(m_motmag.withPosition(11.8 + ZERO_OFFSET));
    m_right.setControl(m_motmag.withPosition(-11.8 - ZERO_OFFSET));
  }
  public void RequestTrap(){
    m_left.setControl(m_motmag.withPosition(1.4 + ZERO_OFFSET));
    m_right.setControl(m_motmag.withPosition(-1.4 - ZERO_OFFSET));
  }
  public void RequestClimb(){
    m_left.setControl(m_motmag.withPosition(7.8 + ZERO_OFFSET));
    m_right.setControl(m_motmag.withPosition(-7.8 - ZERO_OFFSET));
  }

  public double getanglefromdis(double distance){
    
    // return ((-0.311076 * (distance * distance)) + (9.98027 * distance) + -26.5336); //OLD
    return ((-0.143857 * (distance * distance)) + (5.6053 * distance) + 1.81487); //NEW
    
  }


  public double getangleexieriment(double distance){
    SmartDashboard.putNumber("experimentdistance2", distance);
    double angle= 1.6*distance+28.6;
    SmartDashboard.putNumber("experimentangle", angle);
    //return 45;
    return angle;
  }


  public void autoRequestShoot(){
    double distance = m_limelight.getdistance();
    SmartDashboard.putNumber("experimentdistance", distance);

    if (distance > 1) {
      double degrees = getanglefromdis(distance);
            m_left.setControl(m_motmag.withPosition(degrees/10 + ZERO_OFFSET)); //need to find out how the auto target with work
      m_right.setControl(m_motmag.withPosition(-degrees/10 - ZERO_OFFSET));
    } else {
    m_left.setControl(m_motmag.withPosition(3 + ZERO_OFFSET)); //need to find out how the auto target with work
    m_right.setControl(m_motmag.withPosition(-3 - ZERO_OFFSET));
    }


    // m_left.setControl(m_motmag.withPosition(SmartDashboard.getNumber("adushootangle", 0))); //need to find out how the auto target with work
    // m_right.setControl(m_motmag.withPosition(-SmartDashboard.getNumber("adushootangle", 0)));
  }

  public void setAngle(double angle) {
    m_left.setControl(m_motmag.withPosition(angle + ZERO_OFFSET));
    m_right.setControl(m_motmag.withPosition(-angle - ZERO_OFFSET));
  }

  public void smartang(){
      SmartDashboard.putNumber("distance123",  m_limelight.getdistance());

    m_left.setControl(m_motmag.withPosition(SmartDashboard.getNumber("adushootangle", 0)+ZERO_OFFSET)); //need to find out how the auto target with work
    m_right.setControl(m_motmag.withPosition(-SmartDashboard.getNumber("adushootangle", 0)-ZERO_OFFSET));
  }
}
