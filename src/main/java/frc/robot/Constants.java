// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);

  public static class OperatorConstants 
  {
    public static final int kDriverControllerPort = 0;
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }

  public static final class DrivebaseConstants
  {
    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }


  public static class ArmConstants {
    public static final int ARM_MOTOR_ID = 17;

    //Gains for the Arm angle controllers, both FF and PID
    public static final double ARM_KS_ANGLE = 0;
    public static final double ARM_KG_ANGLE = 0.35;
    public static final double ARM_KV_ANGLE = 0;
    public static final double ARM_KP_ANGLE = 0.09;
    public static final double ARM_KI_ANGLE = 0;
    public static final double ARM_KD_ANGLE = 0;    
  }

  public static class TurretConstants {
    public static final int TURRET_MOTOR_ID = 9;

    public static double KP = 0.9; // Proportional gain
    public static double KI = 0.0001; // Integral gain
    public static double KD = 0.0; // Derivative gain
    public static double MAX_OUTPUT = 0.3;
    public static final double MAX_VELOCITY = 1.0; // Max velocity in units/sec
    public static final double MAX_ACCELERATION = 0.5; // Max acceleration in units/sec^2
    public static final double TURRET_GEAR_RATIO = 1.00/15.00;// 15 dev of moter is one rev of turret

  }
  public static class StageConstants {
    public static final int STAGE_MOTOR_ID = 18;

    public static double KP = 0.00005; // Proportional gain
    public static double KI = 0.000001; // Integral gain
    public static double KD = 0.0; // Derivative gain
    public static double MAX_OUTPUT = 1.0;
    public static final double MAX_VELOCITY = 1.0; // Max velocity in units/sec
    public static final double MAX_ACCELERATION = 0.5; // Max acceleration in units/sec^2
  

  }

  public static class LaunchConstants {
    public static final int LAUNCH_MOTOR_ID_A = 12;
    public static final int LAUNCH_MOTOR_ID_B = 11;
    public static double KP = 0.000195;
    public static double KI = 0.0;
    public static double KD = 0.0;
    public static double MAX_OUTPUT = 1;
    public static final double MAX_VELOCITY = 1.0;
    public static final double MAX_ACCELERATION = 0.5 ;
  }
}