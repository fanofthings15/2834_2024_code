// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.ejml.dense.row.MatrixFeatures_CDRM;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class StateManager extends SubsystemBase {

  public Elevator m_Elevator;
  public Wrist    m_Wrist;
  public Intake   m_Intake;
  public LimelightSubsystem m_ll;

  public int elevatorSetPoint = 0;
  public double wristAngle = 0;

  public int test = 0;

  public static enum States{
      HOME,
      INTAKE,
      SHOOT,
      AMP,
      HUMANPLAYER,
      MANUAL
    }

  public StateManager(Wrist wrist, Elevator elevator, Intake intake, LimelightSubsystem ll) {
    m_Elevator = elevator;
    m_Wrist    = wrist;
    m_Intake = intake;
    m_ll = ll;
  }

  
  public void setStates(String state){
    switch (state) {
      
      case "HOME":
        m_Intake.Stop();
        m_Wrist.RequestHome();
        m_Elevator.RequestHome();
      break;

      case "INTAKE":
        m_Elevator.RequestIntake();
        m_Wrist.RequestIntake();
        m_Intake.IntakeIn();
      break;

      case "SHOOT":
        m_Intake.SpinWheels();
        m_Elevator.RequestShoot();
        m_Wrist.RequestShoot();
        // m_Wrist.smartang();
        
      break;

      case "ASHOOT":
        m_Intake.SpinWheels();
        m_Elevator.RequestShoot();
        // m_Wrist.setAngle(m_Wrist.getanglefromdis(m_ll.getdistance()));
        m_Wrist.setAngle(4.8);
      break;

      case "AMP":
        m_Elevator.RequestAmp();
        m_Wrist.RequestAmp();
        m_Intake.Stop();
      break;

      case "HUMANPLAYER":
        m_Elevator.RequestHuman();
        m_Wrist.RequestHuman();
        m_Intake.HumanIntake();
      break;

      case "TRAP":
        m_Elevator.RequestTrap();
        m_Wrist.RequestTrap();
        m_Intake.ShootTrap();
      break;

      case "CLIMBTRAP":
        m_Intake.ShootTrap();

      break;

      case "CLIMB":
        m_Elevator.Requestclib();
        m_Wrist.RequestClimb();
        // m_Intake.ShootTrap();
      break;

      case "PULLUP":
      m_Intake.Stop();
      if(m_Wrist.getAngle()>3){
        m_Elevator.RequestPull();
      }
      break;

      case "CLIMBED":
      m_Intake.Stop();
        if(m_Wrist.getAngle()>3){
          m_Elevator.RequestClimbed();
        }
      break;

      case "GOLONG":
        m_Intake.SpinWheels();
        m_Elevator.RequestHome();
        m_Wrist.setAngle(7.4);
  
      break;

      

      
      case "MANUAL":
      // ?
      break;
  
      default:

        break;
    }

  }
  

  @Override
  public void periodic() {
  }


}
