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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LaunchSubsystem extends SubsystemBase 
{
  // PID Gains and Motion Profile Constraints
  private static double kP = Constants.LaunchConstants.KP;
  private static double kI = Constants.LaunchConstants.KI;
  private static double kD = Constants.LaunchConstants.KD;
  private static double maxOutput = Constants.LaunchConstants.MAX_OUTPUT;
  private double targetVelocity = Constants.LaunchConstants.TARGET_VELOCITY_RPS;
  private boolean isRunning = false;
  
  private SparkMax m_motor_a = new SparkMax(Constants.LaunchConstants.LAUNCH_MOTOR_ID_A, MotorType.kBrushless);
  private SparkMax m_motor_b = new SparkMax(Constants.LaunchConstants.LAUNCH_MOTOR_ID_B, MotorType.kBrushless);
      
  //private SparkMaxConfig m_config = new SparkMaxConfig();
  private SparkMaxConfig m_baseConfig = new SparkMaxConfig();
  private SparkClosedLoopController closedLoopController_a = m_motor_a.getClosedLoopController();
  //private SparkClosedLoopController closedLoopController_b = m_motor_b.getClosedLoopController();
  private RelativeEncoder launchEncoder_a=null;
  //private RelativeEncoder launchEncoder_b=null;


  public LaunchSubsystem() 
  {   
    launchEncoder_a = m_motor_a.getEncoder();      
      
    m_baseConfig.closedLoop
                .p(kP)
                .i(kI)
                .d(kD)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .outputRange((-1 * maxOutput),maxOutput)
                //.feedForward.kV(12.0/917) // set PID 
                ;                        
      m_baseConfig.idleMode(IdleMode.kCoast)
                  .smartCurrentLimit(Constants.LaunchConstants.MAX_CURRENT)
                  .voltageCompensation(Constants.LaunchConstants.MAX_VOLTAGE)
                  ;

      //Update the motoro config to use PID
      m_motor_a.configure(m_baseConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
      //update config for follower
      m_baseConfig.follow(Constants.LaunchConstants.LAUNCH_MOTOR_ID_A,true);
      m_motor_b.configure(m_baseConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  
      SmartDashboard.setDefaultNumber("Launch Target Velocity", 0);
      SmartDashboard.setDefaultBoolean("Launch Run Motor", false);
      SmartDashboard.setDefaultBoolean("Launch Update PID", false);
      SmartDashboard.putNumber("Launch P Gain", kP);
      SmartDashboard.putNumber("Launch I Gain", kI);
      SmartDashboard.putNumber("Launch D Gain", kD);
      SmartDashboard.putNumber("Launch IAccum", 0);
      SmartDashboard.putNumber("Launch Current Velocity", 0);
  }

 public void stop()
  {
    isRunning = false;
    SmartDashboard.putBoolean("Stage Run Motor", false);

    //set the current
    closedLoopController_a.setSetpoint(0, ControlType.kVelocity);
    //turn off motor
    m_motor_a.setVoltage(0);
    //turn off the control mode
  }

  /**
   * Run the launcher to a given velocity
   * @param targetVelocityRPS
   */
  public void runLauncher(double targetVelocityRPS)
  {
    closedLoopController_a.setSetpoint(targetVelocityRPS, ControlType.kVelocity);
  }
   
  /**
   * Runs the launcher to the default configured velocity
   */
  public void runLauncher() 
  {
    if (!isRunning)
    {
      isRunning = true;
      SmartDashboard.putBoolean("Stage Run Motor", true);
    }
    runLauncher(targetVelocity);
  }

  /**
   * Get the current velocity of the motor
   * @return
   */
  public double getVelocity() 
  {
    return launchEncoder_a.getVelocity();
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
  public void periodic() 
  {
    // This method will be called once per scheduler run
    if(!isRunning && (SmartDashboard.getBoolean("Launch Run Motor", false)))
    {
      double targetVelocityDB = SmartDashboard.getNumber("Launch Target Velocity", targetVelocity);
      targetVelocity = targetVelocityDB;
      runLauncher(targetVelocity);
    } 
    else if (!isRunning)
    {
      stop();
    }

    if(SmartDashboard.getBoolean("Launch Update PID", false))
    {
      SmartDashboard.putBoolean("Launch Update PID", false);
            
      //Read PID values
      // read PID coefficients from SmartDashboard
      double p = SmartDashboard.getNumber("Launch P Gain", 0);
      double i = SmartDashboard.getNumber("Launch I Gain", 0);
      double d = SmartDashboard.getNumber("Launch D Gain", 0);
            
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
                    .outputRange((-1 * maxOutput),maxOutput); // set PID and 1/3 max speeds

        //Update the motoro config to use PID
        m_motor_a.configure(m_baseConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
      } //end if updatePID
    } //end update PID
    
    SmartDashboard.putNumber("Launch Current Velocity", getVelocity());
    SmartDashboard.putNumber("Launch IAccum", closedLoopController_a.getIAccum());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
