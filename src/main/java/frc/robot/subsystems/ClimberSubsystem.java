// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.LimitSwitchConfig.Behavior;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.ClimberCommands.StowClimber;

public class ClimberSubsystem extends SubsystemBase 
{
  private SparkMax m_primary_motor = new SparkMax(Constants.Climber.PRIMARY_MOTOR_ID, MotorType.kBrushless);
  private SparkMaxConfig m_climbConfig = new SparkMaxConfig();
  private SparkLimitSwitch m_climberClimbingLimit = null;
  private SparkLimitSwitch m_climberLimitSwitch_reverse = null;
  private SparkMax m_secondary_motor = new SparkMax(Constants.Climber.SECONDARY_MOTOR_ID, MotorType.kBrushless);
  private RelativeEncoder m_climberEncoder;
  //private SparkSoftLimit m_climberClimbingLimit = null;

  private double m_climberPower = Constants.Climber.MAX_POWER;
  private boolean m_isClimberRunning = false;
  private double m_currentPowerLevel = 0;

  private double lastPositionRead = 100;
  private long lastStallReading = 0;
  private boolean m_isStowed = true;
    
  /**
   * Initialize the motor and other components
   */
  public ClimberSubsystem() 
  {     
    //Invert
    m_climbConfig.inverted(Constants.Climber.INVERT);
    m_climbConfig.limitSwitch.forwardLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor);
    m_primary_motor.configure(m_climbConfig, ResetMode.kResetSafeParameters,PersistMode.kNoPersistParameters);

    //Create the limit switches (reverse is limit switch on ground, forward is at climb depth, no forward limit, use rotations)
    m_climberClimbingLimit = m_primary_motor.getForwardLimitSwitch();
    m_climberLimitSwitch_reverse = m_primary_motor.getReverseLimitSwitch();
     
    m_currentPowerLevel = 0;

    //create the encoder from the motor
    m_climberEncoder = m_primary_motor.getEncoder();
    //Track the last time we read the encoder
    lastPositionRead = m_climberEncoder.getPosition();
    //Define current state as being stowed, used in periodic
    m_isStowed = true;

    //For debugging, climber position
    SmartDashboard.putNumber("Climber Current Position",0);
  }

  //This will determine if the motor is stalled
  public boolean isStalled() 
  {
    long currentTime = System.currentTimeMillis();
    long deltaTime = currentTime - lastStallReading;
    boolean isMotorStalled = false;
    //Only check if we are running
    if (!isClimberRunning()) return false;  

    //Check if we are moving at all
    if (deltaTime > 50)
    {
      //check position every 50 ms
      double currentPosition = getCurrentClimberPosition();
      double deltaPosition = currentPosition - lastPositionRead;
      if (deltaPosition < 0.02) isMotorStalled = true;      
      lastStallReading = currentTime;
    }

    //If the motor is supposed to be running, check the current draw
    double motorAppliedOutput = m_primary_motor.getAppliedOutput();
    //TODO determine if we can determine anything from applied output

    return isMotorStalled;
  }
  /**
   * returns true if the intake 4 bar is at reverse limit
   * @return
   */
  public boolean isClimberReverseLimit()
  {
    boolean isReverseLimit =  m_climberLimitSwitch_reverse.isPressed();
    if (isReverseLimit)
    {
      //Zero position
      m_climberEncoder.setPosition(0);
    
      //Set that we are stowed
      m_isStowed = true;
    }

    return isReverseLimit;
  }
  /**
   * returns true if the climber is fully extended
   * @return
   */
  public boolean isAtPrepareToClimbLimit()
  {
    double currentRotations = getCurrentClimberPosition();
    //Return true is we are at or past the number of rotations we determined to be extended
    return (currentRotations >= Constants.Climber.ROTATIONS_FULLY_EXTENDED);
  }

  /**
   * Called for prepare to climb and climb modes, 
   * This will stop the robot from perioically stowing climber
   */
  public void turnOffStow()
  {
    m_isStowed = false;
  }

  /**
   * returns true if the climber is fully extended
   * @return
   */
  public boolean isAtClimbLimit()
  {
    boolean isClimbed = false;
    double currentRotations = getCurrentClimberPosition();
    boolean isForwardLimit =  m_climberClimbingLimit.isPressed();
    //Return true is we are at or past the number of rotations we determined to be at climb or limit is reached
    if (isForwardLimit) isClimbed = true;
    if (currentRotations <= Constants.Climber.ROTATIONS_AT_CLIMB) isClimbed=true; //If we are at or less than the configured climb rots
    return isClimbed;
  }

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
    m_primary_motor.setVoltage(0);
    m_secondary_motor.setVoltage(0);
    m_isClimberRunning = false;
    m_currentPowerLevel = 0;
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
    m_primary_motor.setVoltage(targetVoltage);
    m_secondary_motor.setVoltage(targetVoltage);
    if (targetVoltage != 0) m_isClimberRunning = true; 
    else m_isClimberRunning = false;

    //Update the current power level
     m_currentPowerLevel = targetVoltage;
  }
  /**
   * Run the intake reverse speed
   */
  public void reverseClimber()
  {
    runClimber(-1*m_climberPower);
  }
  public void reverseClimber(double reversePowerLevel)
  {
    runClimber(reversePowerLevel);
  }

  /**
   * Returns the current position from the primary encoder
   * @return
   */
  public double getCurrentClimberPosition()
  { 
    return (m_climberEncoder.getPosition());  
  }

  @Override
  public void periodic() 
  {
    //Update the last position reading
    lastPositionRead = getCurrentClimberPosition();
    SmartDashboard.putNumber("Climber Current Position",lastPositionRead);

    //If we are "stowed" and not currently running the climber ensure our position does not drift too far up
    if ((m_isStowed) && (lastPositionRead >= Constants.Climber.MAX_ROTATIONS_UNDER_BAR) && !isClimberRunning()) 
    {
      //Run the stow command
      System.out.println("Stowing climber again!");
      m_isStowed = false;
      CommandScheduler.getInstance().schedule(new StowClimber(RobotContainer.climberSubsystem));
    }
  }

  @Override
  public void simulationPeriodic() 
  {
    // This method will be called once per scheduler run during simulation
  }
}
