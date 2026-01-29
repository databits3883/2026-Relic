// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class StageSubsystem extends SubsystemBase 
{
  // PID Gains and Motion Profile Constraints
  private static double kP = Constants.StageConstants.KP;
  private static double kI = Constants.StageConstants.KI;
  private static double kD = Constants.StageConstants.KD;
  private static double maxOutput = Constants.StageConstants.MAX_OUTPUT;
  private double targetVelocity = Constants.StageConstants.TARGET_VELOCITY_RPS;
  
  private SparkMax m_motor = new SparkMax(Constants.StageConstants.STAGE_MOTOR_ID, MotorType.kBrushless);
    
  private boolean isRunning = false;      
  //private SparkMaxConfig m_config = new SparkMaxConfig();
  private SparkMaxConfig m_baseConfig = new SparkMaxConfig();
  private SparkClosedLoopController closedLoopController = m_motor.getClosedLoopController();
  private RelativeEncoder stageEncoder=null;


  public StageSubsystem() 
  { 
      stageEncoder = m_motor.getEncoder();   
        
      m_baseConfig.closedLoop
                  .p(kP)
                  .i(kI)
                  .d(kD)
                  .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                  .outputRange((-1 * maxOutput),maxOutput)
                  
                  //.feedForward.kV(12.0/917) // set PID 
                  ;                        
      m_baseConfig.idleMode(IdleMode.kCoast)
                  .smartCurrentLimit(Constants.StageConstants.MAX_CURRENT)
                  .voltageCompensation(Constants.StageConstants.MAX_VOLTAGE)
                  ;

      //Update the motoro config to use PID
      m_motor.configure(m_baseConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
      SmartDashboard.setDefaultNumber("Stage Target Velocity", targetVelocity);
      SmartDashboard.setDefaultBoolean("Stage Run Motor", isRunning);
      SmartDashboard.setDefaultBoolean("Stage Update PID", false);
      SmartDashboard.putNumber("Stage P Gain", kP);
      SmartDashboard.putNumber("Stage I Gain", kI);
      SmartDashboard.putNumber("Stage D Gain", kD);
      SmartDashboard.putNumber("Stage IAccum", 0);
      SmartDashboard.putNumber("Stage Current Velocity", 0);
  }

 public void stop()
  {
    isRunning = false;
    SmartDashboard.putBoolean("Stage Run Motor", false);

    //set the current
    closedLoopController.setSetpoint(0, ControlType.kVelocity);
    closedLoopController.setIAccum(0);
    //turn off motor
    m_motor.setVoltage(0);
    //turn off the control mode
  }

  /**
   * Run the staging motor to the default configured velocity
   */
  public void runStage()
  {
    //Run to the default target speed
    runStage(targetVelocity);
  }
  /** Run the motor to a given speed */
  public void runStage(double targetVelocityRPS)
  {
    if (!isRunning)
    {
      isRunning = true;
      SmartDashboard.putBoolean("Stage Run Motor", true);
    }
    closedLoopController.setSetpoint(targetVelocityRPS, ControlType.kVelocity);
  }

  /**
   * Get the current velocity of the motor
   * @return
   */
  public double getVelocity() 
  {
    return stageEncoder.getVelocity();
  }

  /**
   * Check if we are close enough to our target velocity
   * @return
   */
  public boolean atTargetVelocity() 
  {
    double tolerance = 100.0; // RPM tolerance
    return Math.abs(getVelocity() - targetVelocity) < tolerance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (!isRunning && SmartDashboard.getBoolean("Stage Run Motor", false)) 
    {      
      double targetVelocityDB = SmartDashboard.getNumber("Stage Target Velocity", targetVelocity);
      //Update stored value
      targetVelocity = targetVelocityDB;
      runStage();
    } 
    else if (isRunning)
    {
      stop();
    }

    if(SmartDashboard.getBoolean("Stage Update PID", false))
    {
      SmartDashboard.putBoolean("Stage Update PID", false);
          
      //Read PID values
      // read PID coefficients from SmartDashboard
      double p = SmartDashboard.getNumber("Stage P Gain", 0);
      double i = SmartDashboard.getNumber("Stage I Gain", 0);
      double d = SmartDashboard.getNumber("Stage D Gain", 0);
      
      // if PID coefficients on SmartDashboard have changed, write new values to controller
      boolean updatePID = false;
      if((p != kP)) { updatePID = true;  kP = p; }
      if((i != kI)) { updatePID = true;  kI = i; }
      if((d != kD)) { updatePID = true; kD = d; }

      if (updatePID)
      {
          //Update the PID on close loopController
          m_baseConfig.closedLoop
                  .p(kP)
                  .i(kI)
                  .d(kD)
                  .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                  .outputRange((-1 * maxOutput),maxOutput)
                  ;
          //Update the motoro config to use PID
          m_motor.configure(m_baseConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
      } //end if updatePID
    } // end update stage PID 
    
    SmartDashboard.putNumber("Stage Current Velocity", getVelocity());
    SmartDashboard.putNumber("Stage IAccum", closedLoopController.getIAccum());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
