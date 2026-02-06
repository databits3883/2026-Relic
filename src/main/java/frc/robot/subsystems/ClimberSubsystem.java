// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase 
{
  private SparkMax m_primary_motor = new SparkMax(Constants.Climber.PRIMARY_MOTOR_ID, MotorType.kBrushless);
  private SparkLimitSwitch m_climberLimitSwitch_forward = null;
  private SparkLimitSwitch m_climberLimitSwitch_reverse = null;
  private SparkMax m_secondary_motor = new SparkMax(Constants.Climber.SECONDARY_MOTOR_ID, MotorType.kBrushless);

  private double m_climberPower = Constants.Climber.MAX_POWER;
  //private double m_fourBarPower = Constants.Climber.MAX_POWER;
  private boolean m_isClimberRunning = false;
  //private boolean m_isFourBarRunning = false; //Might not need
    
  public ClimberSubsystem() 
  { 
      m_climberLimitSwitch_forward = m_primary_motor.getForwardLimitSwitch();
      m_climberLimitSwitch_reverse = m_primary_motor.getReverseLimitSwitch();
  }

  /**
   * returns true if the intake 4 bar is at reverse limit
   * @return
   */
  public boolean isClimberReverseLimit()
  {
    return m_climberLimitSwitch_reverse.isPressed();
  }
  /**
   * returns true if the intake 4 bar is at forward limit
   * @return
   */
  public boolean isClimberForwardLimit()
  {
    return m_climberLimitSwitch_forward.isPressed();
  }
  /**
   * Returns true if the motor is spinning
   * @return
   */

  

  /**
   * Returns true if the intake motor is running
   * @return
   */
  public boolean isClimberRunning()
  {
    return m_isClimberRunning;
  }

  /**
   * Stop the intake from spinning
   */
  public void stopClimber()
  {
    //turn off motor
    m_secondary_motor.setVoltage(0);
    m_isClimberRunning = false;
  }

  /**
   * Run the intake motor to the default configured velocity
   */
  public void runClimber()
  {
    //Run to the default target power
    runClimber(m_climberPower);
  }
  /** Run the motor to a given power */
  public void runClimber(double targetVoltage)
  {
    m_secondary_motor.setVoltage(targetVoltage);
    if (targetVoltage != 0) m_isClimberRunning = true; 
    else m_isClimberRunning = false;
  }
  /**
   * Run the intake reverse speed
   */
  public void reverseClimber()
  {
    runClimber(-1*m_climberPower);
  }

  @Override
  public void periodic() 
  {
    //Nothing to do here at the moment
  }

  @Override
  public void simulationPeriodic() 
  {
    // This method will be called once per scheduler run during simulation
  }
}
