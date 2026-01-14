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
    public static final int kDriverControllerPort = 0;
  }

  public static class ArmConstants {
    public static final int ARM_MOTOR_ID = 17;

    //Gains for the Arm angle controllers, both FF and PID
    public static final double ARM_KS_ANGLE = 0;
    public static final double ARM_KG_ANGLE = 0.35;//0.085 gains for no coral and set()  // 0.35
    public static final double ARM_KV_ANGLE = 0;
    public static final double ARM_KP_ANGLE = 0.09;//0.0025 gains for no coral and set() //0.04 //0.09
    public static final double ARM_KI_ANGLE = 0;//0.001 gains for no coral and set()
    public static final double ARM_KD_ANGLE = 0;    
  }
}
