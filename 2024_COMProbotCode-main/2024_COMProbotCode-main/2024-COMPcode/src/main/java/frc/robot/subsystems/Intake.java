// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  TalonFX m_Intake;
  TalonFX m_Feeder;


  //13 is intaking height 0 is inside bot

  public Intake(){
    m_Intake = new TalonFX(Constants.IntakeConstants.IntakeID , "Canivore"); //change IDS here or on motors
    m_Feeder = new TalonFX(Constants.IntakeConstants.FeederID , "Canivore");
  }

  public void IntakeIn(){ //Change To Negitive values if needed
    m_Intake.set(.55);
    m_Feeder.set(.1);
  }
  public void HumanIntake(){ //Change To Negitive values if needed
    m_Intake.set(.3);
    m_Feeder.set(.1);
  }
  public void IntakeOut(){//Change To Positive values if needed
    m_Intake.set(-.5);
    new WaitCommand(.3);
    m_Feeder.set(-.1);
  }
  public void SpinWheels(){//Change To Negitive values if needed
    m_Intake.set(-.9);//-.8
  }
  public void ShootTrap(){//Change To Negitive values if needed
    m_Intake.set(-.15);//-.8
  }
  public void Feed(){//Change To Negitive values if needed
    m_Feeder.set(-.3);
  }
  public void Stop(){
    m_Intake.stopMotor();
    m_Feeder.stopMotor();
  }
  public void SpinIntakeTo(double speed){
    m_Intake.set(speed);
  }
  
  @Override
  public void periodic() {

  }
}