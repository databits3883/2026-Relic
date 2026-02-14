// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
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
    //Set up the initial poses
    public static final Pose2d INITITAL_RED_POSE = new Pose2d(new Translation2d(Meter.of(16),
                                                                      Meter.of(4)),
                                                    Rotation2d.fromDegrees(180));
    public static final Pose2d INITITAL_BLUE_POSE = new Pose2d(Units.inchesToMeters(18.5),Units.inchesToMeters(251.84),
                                                    Rotation2d.fromDegrees(0));
  }

  public static class TurretConstants {
    public static final int TURRET_MOTOR_ID = 9;

    //OLD kp that was working, but might have smoked motor
    public static double KP = 5; // Proportional gain
    public static double KI = 0.00008; // Integral gain
    public static double KD = 0.0; // Derivative gain
    public static double MAX_OUTPUT = 0.3;
    public static final double MAX_VELOCITY = 1.0; // Max velocity in units/sec
    public static final double MAX_ACCELERATION = 0.5; // Max acceleration in units/sec^2
    public static final double TURRET_GEAR_RATIO = 1.00/14.00;// 15 dev of moter is one rev of turret
    public static final double ALIGNMENT_SWITCH_ANGLE = 360-40; // The angle that the alignment switch sets the turret at
    public static final double START_TURRET_ANGLE = 0; //The angle the turret starts in
    public static final double MAX_ANGLE_ERROR = 15; //If the turret is off by this or more when the switch is hit then update the angle

    public static final double MID_FIELD_Y = Units.inchesToMeters(158.84);
    public static final double RED_X_PLAYER = Units.inchesToMeters(651.22 - 182.11 );
    public static final double BLUE_X_PLAYER = Units.inchesToMeters(182.11);

    public static final Pose2d RED_HUB_POSE = new Pose2d(RED_X_PLAYER, MID_FIELD_Y, new Rotation2d(0)); 
    public static final Pose2d RED_BOTTOM_POSE = new Pose2d(RED_X_PLAYER+(BLUE_X_PLAYER/2), (MID_FIELD_Y * 0.5), new Rotation2d(0)); 
    public static final Pose2d RED_TOP_POSE = new Pose2d(RED_BOTTOM_POSE.getX(), (MID_FIELD_Y * 1.5), new Rotation2d(0)); 
    public static final Pose2d BLUE_BOTTOM_POSE = new Pose2d((BLUE_X_PLAYER/2), RED_BOTTOM_POSE.getY(), new Rotation2d(0)); 
    public static final Pose2d BLUE_HUB_POSE = new Pose2d(BLUE_X_PLAYER, MID_FIELD_Y, new Rotation2d(0)); 
    public static final Pose2d BLUE_TOP_POSE = new Pose2d((BLUE_X_PLAYER/2), RED_TOP_POSE.getY(), new Rotation2d(0)); 

    //public static final Transform2d CENTER_TURRET_FROM_CENTER_BOT_BACK = new Transform2d(Units.inchesToMeters(-7),0,new Rotation2d(0));
    //public static final Transform2d CENTER_TURRET_FROM_CENTER_BOT_FRONT = new Transform2d(Units.inchesToMeters(7),0,new Rotation2d(0));
    //public static final Transform2d CENTER_TURRET_FROM_CENTER_BOT_LEFT = new Transform2d(0,Units.inchesToMeters(7),new Rotation2d(0));
    //public static final Transform2d CENTER_TURRET_FROM_CENTER_BOT_RIGHT = new Transform2d(0,Units.inchesToMeters(-7),new Rotation2d(0));
    //public static final Transform2d BACK_LEFT_TURRET_FROM_CENTER_BOT = new Transform2d(Units.inchesToMeters(6.125),Units.inchesToMeters(3.25),new Rotation2d(0));
    public static final Transform2d BACK_LEFT_TURRET_FROM_CENTER_BOT = new Transform2d(Units.inchesToMeters(3.25),Units.inchesToMeters(6.125),new Rotation2d(0));
    public static final double TURRET_ANGLE_OFFSET = 20; //Offset all targets by 30 degrees clockwise
  }
  
  public static class Intake 
  {
    public static final int FOUR_BAR_MOTOR_ID = 13;
    public static double FOUR_BAR_FORWARD_POWER = 2.0;
    public static int FOUR_BAR_FORWARD_TIMEOUT_SEC = 1; /* Max run time in case we do not hit limit */
    public static double FOUR_BAR_BACKWARD_POWER = 4.0;
    public static int FOUR_BAR_BACKWARD_TIMEOUT_SEC = 4; /* Max run time in case we do not hit limit */

    public static final int INTAKE_MOTOR_ID = 14;
    public static double INTAKE_MOTOR_POWER = 14.0;
    public static boolean INTAKE_MOTOR_INVERSE = true;
  }

  public static class Climber
  {
    public static final int PRIMARY_MOTOR_ID = 16;
    /* Secondary will follow the primary, just adding id for clarrity */
    public static final int SECONDARY_MOTOR_ID = 17; 
    
    public static double MAX_POWER = 10.0;
    public static double SLOW_REVERSE_SPEED = -0.3;

    public static int CLIMBER_TIMEOUT_SEC = 4;
    public static int STOW_TIMEOUT_SEC = 4;
  }

  public static class StageConstants {
    public static final int STAGE_MOTOR_ID = 10;
    public static boolean STAGE_INVERSE = true;
    public static boolean STAGE_USE_PID = false;

    public static double KP = 0.0004; // Proportional gain
    public static double KI = 0.0000009; // Integral gain
    public static double KD = 0.000001; // Derivative gain
    public static double MAX_OUTPUT = 1.0;
    public static final double MAX_ACCELERATION = 0.5; // Max acceleration in units/sec^2
    public static final int MAX_CURRENT = 40; //amps
    public static final double MAX_VOLTAGE = 14;  // Max voltage when in non-pid mode
    public static final double TARGET_VELOCITY_RPS = 7000; 
    public static final double TOLERANCE = 100; //rps

    public static final int SPINDEXER_MOTOR_ID = 15;
    public static double SPINDEXER_MOTOR_POWER = 3.0;
    public static boolean SPINDEXER_INVERSE = true;

    public static final int OMNIDEXER_MOTOR_ID = 18;
    public static double OMNIDEXER_MOTOR_POWER = 3.0;
  }

  public static class LaunchConstants {
    public static final int LAUNCH_MOTOR_ID_A = 11;
    public static final int LAUNCH_MOTOR_ID_B = 12;
    public static double KP = 0.00031;
    public static double KI = 0.0000009;
    public static double KD = 0.0;
    public static double MAX_OUTPUT = 1;
    public static final double MAX_VELOCITY = 1.0;
    public static final double MAX_ACCELERATION = 0.5 ;
    public static final int MAX_CURRENT = 60; //amps
    public static final double MAX_VOLTAGE = 12; //volts
    public static final double TARGET_VELOCITY_RPM = 2700; 
    public static final double TOLERANCE = 100; //rps

    public static final double MAX_SHOOTING_DISTANCE = 5; //Max distance we can shoot at max velocity in meters
    public static final double MIN_SHOOTING_MIN_VELOCITY_RPM = 1000;
  }

}