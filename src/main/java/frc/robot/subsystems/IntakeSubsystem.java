// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase 
{
  private SparkMax m_four_bar_motor = new SparkMax(Constants.Intake.FOUR_BAR_MOTOR_ID, MotorType.kBrushless);
  private SparkMaxConfig m_intakeConfig = new SparkMaxConfig();

  private SparkLimitSwitch m_fouSparkLimitSwitch_forward = null;
  private SparkLimitSwitch m_fouSparkLimitSwitch_reverse = null;
  private SparkMax m_intake_motor = new SparkMax(Constants.Intake.INTAKE_MOTOR_ID, MotorType.kBrushless);
  private RelativeEncoder m_intakeEncoder = null;

  private double m_intakeSpinningPower = Constants.Intake.INTAKE_MOTOR_POWER;
  private double m_fourBarForwardPower = Constants.Intake.FOUR_BAR_FORWARD_POWER;
  private double m_fourBarBackwardPower = Constants.Intake.FOUR_BAR_BACKWARD_POWER;
  
  private boolean m_isIntakeRunning = false;
  private boolean m_isFourBarRunning = false;
  private boolean m_isIntakeDeployed = false;

  //Velocity Control
  private SparkClosedLoopController closedLoopController = m_intake_motor.getClosedLoopController();
  private static double maxOutput = Constants.Intake.MAX_OUTPUT;
  private static double SLOT1_kP = Constants.Intake.SLOT1_KP;
  private static double SLOT1_kI = Constants.Intake.SLOT1_KI;
  private static double SLOT1_kD = Constants.Intake.SLOT1_KD;
  private static double SLOT1_kV = Constants.Intake.SLOT1_kV;
  private double currentSetPointRPM = 0;
  private double defaultSetPointRPM =  Constants.Intake.TARGET_VELOCITY_RPM;



  //For isStalled method
  private long intakeLastStallReading = 0;
  private double intakeStalledLastPositionRead = 0;
    
  public IntakeSubsystem() 
  { 
      m_fouSparkLimitSwitch_forward = m_four_bar_motor.getForwardLimitSwitch();
      m_fouSparkLimitSwitch_reverse = m_four_bar_motor.getReverseLimitSwitch();
      m_intakeConfig.idleMode(IdleMode.kCoast).inverted(Constants.Intake.INTAKE_MOTOR_INVERSE);

      //Set up PID for velocity control
       m_intakeConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .outputRange((-1 * maxOutput),maxOutput)
                .pid(SLOT1_kP, SLOT1_kI, SLOT1_kD,ClosedLoopSlot.kSlot1) // slot 1, feed forward
                .feedForward
                  .kS(0.0,ClosedLoopSlot.kSlot1) // slot 1
                  .kV(SLOT1_kV, ClosedLoopSlot.kSlot1) // slot 1
                ;                        

      //Update the motor config
      m_intake_motor.configure(m_intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

      intakeLastStallReading = System.currentTimeMillis();
      m_intakeEncoder = m_intake_motor.getEncoder();
      intakeStalledLastPositionRead = getIntakePosition();

      SmartDashboard.setDefaultNumber("Intake Target Velocity", defaultSetPointRPM);
      SmartDashboard.setDefaultBoolean("Intake Update PID", false);
      SmartDashboard.putNumber("Intake P Gain", SLOT1_kP);
      SmartDashboard.putNumber("Intake I Gain", SLOT1_kI);
      SmartDashboard.putNumber("Intake D Gain", SLOT1_kD);
      SmartDashboard.putNumber("Intake V Gain", SLOT1_kV);
      SmartDashboard.putNumber("Intake Current Velocity",0);
  }

  /**
   * This will check if the intake is stalled, it checks every 50ms if it has moved 
   * @return boolean
   */
  public boolean isIntakeStalled() 
  {
    long currentTime = System.currentTimeMillis();
    long deltaTime = currentTime - intakeLastStallReading;
    boolean isMotorStalled = false;
    //Only check if we are running
    if (!isIntakeRunning()) return false;  

    //Check if we are moving at all
    if (deltaTime > Constants.Intake.INTAKE_STALL_CHECK_MS)
    {
      //check position every 50 ms
      double currentPosition = getIntakePosition();
      double deltaPosition = currentPosition - intakeStalledLastPositionRead;
      if (Math.abs(deltaPosition) < Constants.Intake.INTAKE_MIN_POSITION_MOVEMENT) isMotorStalled = true;  
      //Update last readings    
      intakeLastStallReading = currentTime;
      intakeStalledLastPositionRead = currentPosition;      

      if (isMotorStalled)
        System.out.print("Intake Stalled!!");
    }

    return isMotorStalled;
  }

  /**
   * Returns the position of the encoder
   * @return
   */
  private double getIntakePosition()
  {
    return m_intakeEncoder.getPosition();
  }


  /**
   * Used to stop the intake by a button
   */
  public void overrideStopIntake()
  {
    if (isIntakeRunning())
    {
      stopIntake();
    }
  }
  /**
   * Used to start the intake (if deployed) by button
   */
  public void overrideStartIntake()
  {
    if (!isIntakeRunning())
    {
      runIntake();
    }
  }

  /**
   * returns true if the intake 4 bar is at reverse limit
   * @return
   */
  public boolean isFourBarReverseLimit()
  {
    boolean isAtLimit =  m_fouSparkLimitSwitch_reverse.isPressed();
    //Set the opposite of the limit switch for intake deployed state
    intakeDeplyed(!isAtLimit);
    return isAtLimit;
  }
  /**
   * returns true if the intake 4 bar is at forward limit
   * @return
   */
  public boolean isFourBarForwardLimit()
  {
    boolean isAtLimit =  m_fouSparkLimitSwitch_forward.isPressed();
    intakeDeplyed(isAtLimit);
    return isAtLimit;
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
    runFourBar(m_fourBarForwardPower);
  }
  /** Run the motor to a given power */
  public void runFourBar(double targetVoltage)
  {
    m_four_bar_motor.setVoltage(targetVoltage);
    if (targetVoltage != 0) m_isFourBarRunning = true; 
    else m_isFourBarRunning = false;
  }
  /**
   * Run the four bar reverse speed
   */
  public void reverseFourBar()
  {
    runFourBar(-1*m_fourBarBackwardPower);
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
   * Return if the intake four bar system is deployed or not
   * @return
   */
  public boolean isIntakeDeployed()
  {
    return m_isIntakeDeployed;
  }

  /**
   * Update the four bar state, deployed or retracted
   * @param fourBarState
   */
  public void intakeDeplyed(boolean fourBarState)
  {
    if (m_isIntakeDeployed != fourBarState)
    {
      //State of intake has changed
      m_isIntakeDeployed = fourBarState;
      //try to run the intake
      if (fourBarState) runIntake();
      else stopIntake();
    }
  }

  /**
   * Stop the intake from spinning
   */
  public void stopIntake()
  {
    //turn off motor
    m_intake_motor.setVoltage(0);

    closedLoopController.setSetpoint(0, ControlType.kVelocity,ClosedLoopSlot.kSlot1);
    currentSetPointRPM = 0;

    m_isIntakeRunning = false;
  }

  /**
   * Run the intake motor to the default configured velocity
   */
  public void runIntake()
  {
    //Run to the default target power
    runIntake(defaultSetPointRPM);
  }
  public void runIntake(double targetVelocityRPM)
  {
    //Only update the launcher if the speed changes
    if (currentSetPointRPM != targetVelocityRPM)
    {
      currentSetPointRPM = targetVelocityRPM;
      SmartDashboard.putNumber("Intake Target Velocity", targetVelocityRPM);
      closedLoopController.setSetpoint(targetVelocityRPM*1.1, ControlType.kVelocity,ClosedLoopSlot.kSlot1);
    }
  }

  /** Run the motor to a given power */
/*   public void runPowerIntake(double targetVoltage)
  {
    //Only allow the intake to run if deployed
    if (isIntakeDeployed())
    {
      m_intake_motor.setVoltage(targetVoltage);
      if (targetVoltage != 0) m_isIntakeRunning = true; 
      else m_isIntakeRunning = false;
    }
  }*/
  /**
   * Run the intake reverse speed
   */
  public void reverseIntake()
  {
    runIntake(-1*defaultSetPointRPM);
  }

  /**
   * Gets the velocity of the intake
   * @return
   */
  public double getVelocityPrimary() 
  {
    return m_intakeEncoder.getVelocity();
  }


  @Override
  public void periodic() 
  {
    //Add stall check, need to verify this works as expected
    if (isIntakeRunning() && isIntakeStalled())
    {
      stopIntake();
    }

    if (SmartDashboard.getBoolean("Intake Update PID", false))
    {
      SmartDashboard.putBoolean("Intake Update PID", false);
            
      //Read PID values
      // read PID coefficients from SmartDashboard
      double p = SmartDashboard.getNumber("Intake P Gain", 0);
      double i = SmartDashboard.getNumber("Intake I Gain", 0);
      double d = SmartDashboard.getNumber("Intake D Gain", 0);
      double v = SmartDashboard.getNumber("Intake V Gain", 0);
      double velocity = SmartDashboard.getNumber("Intake Target Velocity", Constants.Intake.TARGET_VELOCITY_RPM);
            
      // if PID coefficients on SmartDashboard have changed, write new values to controller
      boolean updatePID = false;
      if((p != SLOT1_kP)) { updatePID = true;  SLOT1_kP = p; }
      if((i != SLOT1_kI)) { updatePID = true;  SLOT1_kI = i; }
      if((d != SLOT1_kD)) { updatePID = true; SLOT1_kD = d; }
      if((v != SLOT1_kV)) { updatePID = true; SLOT1_kV = v; }
      if (defaultSetPointRPM != velocity) defaultSetPointRPM = velocity;

      if (updatePID)
      {
        //Update the PID on close loopController
        m_intakeConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    .outputRange((-1 * maxOutput),maxOutput) // set PID 
                    .pid(SLOT1_kP, SLOT1_kI, SLOT1_kD,ClosedLoopSlot.kSlot1) // slot 1, feed forward
                    .feedForward
                      .kS(0.0,ClosedLoopSlot.kSlot1) // slot 1
                      .kV(SLOT1_kV, ClosedLoopSlot.kSlot1) // slot 1
                    ;

        //Update the motoro config to use PID
        m_intake_motor.configure(m_intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
      } //end if updatePID
      if (isIntakeRunning()) runIntake(defaultSetPointRPM);

    } //end update PID
    
    SmartDashboard.putNumber("Intake Current Velocity",getVelocityPrimary());
  }

  @Override
  public void simulationPeriodic() 
  {
    // This method will be called once per scheduler run during simulation
  }
}
