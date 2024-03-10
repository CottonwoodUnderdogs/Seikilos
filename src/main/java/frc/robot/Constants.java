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
  public static class OperatorConstants {
    // public static class SwitchMappings {
    //   public static final int kDriverControllerPort = 0;
    //   public static final int controllerXAxis   = 1;        // Mappings for Switch Controller
    //   public static final int controllerYAxis   = 0;        // Axes labeled by mecanum direction, not actual axis.
    //   public static final int controllerZAxis   = 2;
    //   public static final int controllerZRotate = 3;

    //   public static final int Y  = 1;
    //   public static final int B  = 2;
    //   public static final int A  = 3;
    //   public static final int X  = 4;
    //   public static final int L  = 5;
    //   public static final int R  = 6;
    //   public static final int ZL = 7;
    //   public static final int ZR = 8;

    //   public static final int SELECT = 9;
    //   public static final int START  = 10;
    // }
    public static class XboxMappings {
      public static final int PRIMARY_CONTROLLER_PORT = 0;
      public static final int SECONDARY_CONTROLLER_PORT = 1;
      public static final int LXAxis = 0; // wow xbox controller axes are way better named than the switch ones.
      public static final int LYAxis = 1;
      public static final int RXAxis = 4; 
      public static final int RYAxis = 5;

      public static final int A = 1;
      public static final int B = 2;
      public static final int X = 3;
      public static final int Y = 4;
      public static final int L = 5;
      public static final int R = 6;

    }
    
  }
  public static class MotorID {                
    public static final int FRONT_RIGHT = 1;   // Driving Motors
    public static final int FRONT_LEFT  = 3;
    public static final int BACK_RIGHT  = 2;
    public static final int BACK_LEFT   = 4;

    public static final int SHOOTER = 5;       // Shooting Motors
    public static final int SHOOTER2 = 11;
    public static final int FEEDER  = 6;

    public static final int ANGLER = 7;
    
    public static final int CLIMBER_LEFT  = 10; // Climbing Motors
    public static final int CLIMBER_RIGHT = 8;

    public static final int COLLECTOR = 9;
  }
  
  public static class MotorSpeeds {
    // Tele-op speeds
    public static final double FEEDER_SPEED = 0.8;
    public static final double SHOOTER_SPEED = 0.2;
    public static final double COLLECTOR_SPEED = 0.6;
    public static final double DRIVE_SPEED = 0.9;
    public static final double DRIVE_SLOW_SPEED = 0.3;
    public static final double CLIMBER_LEFT_SPEED = 0.8;
    public static final double CLIMBER_RIGHT_SPEED = 0.8;
    // Auto speeds
    public static final double DRIVE_AUTO_SPEED = 0.6;
  }

  public static class Inputs {
    // Beam break sensors
    public static final int NOTE_LOADED_CHANNEL = 0;
    // Limit Switches
    public static final int LEFT_CLIMBER_CHANNEL = 2;
    public static final int RIGHT_CLIMBER_CHANNEL = 3;
  }

  public static class SetPoints {
    public static final double ANGLER_DIRECT = -14;
    public static final double ANGLER_SMALL_DISTANCE = -9;


  }
}
