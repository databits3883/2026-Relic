// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase 
{
  private SparkMax m_four_bar_motor = new SparkMax(Constants.Intake.FOUR_BAR_MOTOR_ID, MotorType.kBrushless);
  private SparkLimitSwitch m_fouSparkLimitSwitch_forward = null;
  private SparkLimitSwitch m_fouSparkLimitSwitch_reverse = null;
  private SparkMax m_intake_motor = new SparkMax(Constants.Intake.INTAKE_MOTOR_ID, MotorType.kBrushless);

  private double m_intakeSpinningPower = Constants.Intake.INTAKE_MOTOR_POWER;
  private double m_fourBarPower = Constants.Intake.FOUR_BAR_POWER;
  private boolean m_isIntakeRunning = false;
  private boolean m_isFourBarRunning = false;
    
  public IntakeSubsystem() 
  { 
      m_fouSparkLimitSwitch_forward = m_four_bar_motor.getForwardLimitSwitch();
      m_fouSparkLimitSwitch_reverse = m_four_bar_motor.getReverseLimitSwitch();
  }

  /**
   * returns true if the intake 4 bar is at reverse limit
   * @return
   */
  public boolean isFourBarReverseLimit()
  {
    return m_fouSparkLimitSwitch_reverse.isPressed();
  }
  /**
   * returns true if the intake 4 bar is at forward limit
   * @return
   */
  public boolean isFourBarForwardLimit()
  {
    return m_fouSparkLimitSwitch_forward.isPressed();
  }
  /**
   * Returns true if the motor is spinning
   * @return
   */
  public boolean isFourBarRunning()
  {
    return m_isFourBarRunning;
  }

    /**
   * Run the four bar motor to the default configured velocity
   */
  public void runFourBar()
  {
    //Run to the default target power
    runFourBar(m_fourBarPower);
  }
  /** Run the motor to a given power */
  public void runFourBar(double targetVoltage)
  {
    m_intake_motor.setVoltage(targetVoltage);
    if (targetVoltage != 0) m_isFourBarRunning = true; 
    else m_isFourBarRunning = false;
  }
  /**
   * Run the four bar reverse speed
   */
  public void reverseFourBar()
  {
    runFourBar(-1*m_fourBarPower);
  }

  /**
   * Stop the intake from spinning
   */
  public void stopFourBar()
  {
    //turn off motor
    m_four_bar_motor.setVoltage(0);
    m_isFourBarRunning = false;
  }

  /**
   * Returns true if the intake motor is running
   * @return
   */
  public boolean isIntakeRunning()
  {
    return m_isIntakeRunning;
  }

  /**
   * Stop the intake from spinning
   */
  public void stopIntake()
  {
    //turn off motor
    m_intake_motor.setVoltage(0);
    m_isIntakeRunning = false;
  }

  /**
   * Run the intake motor to the default configured velocity
   */
  public void runIntake()
  {
    //Run to the default target power
    runIntake(m_intakeSpinningPower);
  }
  /** Run the motor to a given power */
  public void runIntake(double targetVoltage)
  {
    m_intake_motor.setVoltage(targetVoltage);
    if (targetVoltage != 0) m_isIntakeRunning = true; 
    else m_isIntakeRunning = false;
  }
  /**
   * Run the intake reverse speed
   */
  public void reverseIntake()
  {
    runIntake(-1*m_intakeSpinningPower);
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
