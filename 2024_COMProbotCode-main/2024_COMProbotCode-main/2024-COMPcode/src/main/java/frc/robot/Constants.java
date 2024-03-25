// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {


  public static class Driver {
    public static final int  ControllerPort = 0;
    public static final int  zeroGyro = 1;
    public static final int  AutoAIm = 2;
  }


  public static class ButtonBox1 {
    public static final int BoxPort = 1;

    public static final int State_HOME        = 1;
    public static final int State_INTAKE      = 9;
    public static final int State_SHOOT       = 8;
    public static final int State_AMP         = 5;
    public static final int State_HUMANPLAYER = 4;
    public static final int State_TRAP        = 6;
    public static final int State_clib        = 7;
    

    // public static final int elevUP   = 7;
    // public static final int elevDOWN = 8;

    public static final int wristUP    = 9;
    public static final int writstDOWN = 10;

    public static final int Outtake  = 10;

    public static final int GoLong  = 11;

    public static final int reZeroElev  = 12;
    public static final int reZeroWrist = 13;

    //Climb Buttos auto and manual

  }

    public static class ButtonBox2 {
      public static final int wristup = 3;
      public static final int wristdown = 2;
      public static final int elevup = 4;
      public static final int elevdown = 1;

      public static final int climbDown = 9;
      public static final int FullClimed = 5;
      public static final int ClimbTrap = 12;

      //DO not touch
      public static final int DNT = 11;
      
      
    }

  public static class ElevatorConstants {
    public static final int LEFTID  = 31;
    public static final int RIGHTID = 32;
  }

  public static class WristConstants {
    public static final int LEFTID  = 33;
    public static final int RIGHTID = 34;
  }

  public static class IntakeConstants {
    public static final int IntakeID  = 35;
    public static final int FeederID  = 36;
  }

  public static class DriveConstats{
    // private static final int kFrontLeftDriveMotorId = 2;
    // private static final int kFrontLeftSteerMotorId = 4;
    // private static final int kFrontLeftEncoderId = 26

    // private static final int kFrontRightDriveMotorId = 6;
    // private static final int kFrontRightSteerMotorId = 5;
    // private static final int kFrontRightEncoderId = 22;

    // private static final int kBackLeftDriveMotorId = 7;
    // private static final int kBackLeftSteerMotorId = 8;
    // private static final int kBackLeftEncoderId = 28;

    // private static final int kBackRightDriveMotorId = 3;
    // private static final int kBackRightSteerMotorId = 1;
    // private static final int kBackRightEncoderId = 24;

  }
}
