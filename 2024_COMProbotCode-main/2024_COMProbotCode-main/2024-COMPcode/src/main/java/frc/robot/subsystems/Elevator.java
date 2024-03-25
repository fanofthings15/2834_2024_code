// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  TalonFX m_left;
  TalonFX m_right;
  ProfiledPIDController m_pid;
  Constraints trappro;
  MotionMagicVoltage m_motmag;
  
  public Elevator() {
    m_right = new TalonFX(Constants.ElevatorConstants.RIGHTID , "Canivore");
    m_left = new TalonFX(Constants.ElevatorConstants.LEFTID , "Canivore");


    m_motmag = new MotionMagicVoltage(0);

    var talonFXConfigs = new TalonFXConfiguration();
    
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = 0.24; // add 0.24 V to overcome friction
    slot0Configs.kV = 0.12; // apply 12 V for a target velocity of 100 rps
    // PID runs on position

    slot0Configs.kP = 2; //1
    slot0Configs.kI = 0;
    slot0Configs.kD = 0;

    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 80; // 80 rps cruise velocity
    motionMagicConfigs.MotionMagicAcceleration = 120; // 160 rps/s acceleration (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 600; // 1600 rps/s^2 jerk (0.1 seconds)

    m_left.getConfigurator().apply(talonFXConfigs, 0.050);
    m_right.getConfigurator().apply(talonFXConfigs, 0.050);
    // periodic, run Motion Magic with slot 0 configs,
  }

  

  @Override
  public void periodic() {
    m_motmag.Slot = 0;
    SmartDashboard.putNumber("intake height", getpose());
  }

public Double getpose(){
  return (m_left.getPosition().getValue() + m_right.getPosition().getValue()) / 2;
}
  

//-50 is top 0 is bottom


  
  public void RequestHome(){
    m_left.setControl(m_motmag.withPosition(-0));
    m_right.setControl(m_motmag.withPosition(-0));
  }

   public void RequestIntake(){
    m_left.setControl(m_motmag.withPosition(0));
    m_right.setControl(m_motmag.withPosition(0));
  }

  public void RequestShoot(){
    m_left.setControl(m_motmag.withPosition(-5));
    m_right.setControl(m_motmag.withPosition(-5));
  }
  
  public void RequestHuman(){
    m_left.setControl(m_motmag.withPosition(-21.8));
    m_right.setControl(m_motmag.withPosition(-21.8));
  }

  public void RequestAmp(){
    m_left.setControl(m_motmag.withPosition(-53));
    m_right.setControl(m_motmag.withPosition(-53));
  }

  public void RequestTrap(){
    m_left.setControl(m_motmag.withPosition(-35));
    m_right.setControl(m_motmag.withPosition(-35));
  }

  public void Requesttest(){
    m_left.setControl(m_motmag.withPosition(-19));
    m_right.setControl(m_motmag.withPosition(-19));
  }

  public void Requestclib(){
    m_left.setControl(m_motmag.withPosition(-57));
    m_right.setControl(m_motmag.withPosition(-57));
  }

  public void RequestPull(){
    m_left.setControl(m_motmag.withPosition(-48));
    m_right.setControl(m_motmag.withPosition(-48));
  }

  public void RequestClimbed(){
    m_left.setControl(m_motmag.withPosition(-24));
    m_right.setControl(m_motmag.withPosition(-24));
  }

  public void setheight(double height) {
    m_left.setControl(m_motmag.withPosition(height));
    m_right.setControl(m_motmag.withPosition(height));
  }
  
}
