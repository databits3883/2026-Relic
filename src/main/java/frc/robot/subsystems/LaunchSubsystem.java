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
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class LaunchSubsystem extends SubsystemBase 
{
  // PID Gains and Motion Profile Constraints
  private static double SLOT0_kP = Constants.LaunchConstants.SLOT0_KP;
  private static double SLOT0_kI = Constants.LaunchConstants.SLOT0_KI;
  private static double SLOT0_kD = Constants.LaunchConstants.SLOT0_KD;
  private static double maxOutput = Constants.LaunchConstants.MAX_OUTPUT;
  private static double SLOT1_kP = Constants.LaunchConstants.SLOT1_KP;
  private static double SLOT1_kI = Constants.LaunchConstants.SLOT1_KI;
  private static double SLOT1_kD = Constants.LaunchConstants.SLOT1_KD;
  private static double SLOT1_kV = Constants.LaunchConstants.SLOT1_kV;
  private double currentSetPointRPM = 0;
  private double defaultSetPointRPM =  Constants.LaunchConstants.TARGET_VELOCITY_RPM;
  private boolean isRunning = false;
  private boolean x_useTargetDistance = true;
  private boolean x_useSlot0 = true;
  
  private SparkFlex m_motor_a = new SparkFlex(Constants.LaunchConstants.LAUNCH_MOTOR_ID_A, MotorType.kBrushless);
  private SparkFlex m_motor_b = new SparkFlex(Constants.LaunchConstants.LAUNCH_MOTOR_ID_B, MotorType.kBrushless);

  //private SparkMaxConfig m_config = new SparkMaxConfig();
  private SparkFlexConfig m_baseConfig_a = new SparkFlexConfig();
  private SparkFlexConfig m_baseConfig_b = new SparkFlexConfig();
  private SparkClosedLoopController closedLoopController_a = m_motor_a.getClosedLoopController();
  //private SparkClosedLoopController closedLoopController_b = m_motor_b.getClosedLoopController();
  private RelativeEncoder launchEncoder_a=null;

  public LaunchSubsystem() 
  {   
    launchEncoder_a = m_motor_a.getEncoder();      
      
    m_baseConfig_a.closedLoop
                .p(SLOT0_kP,ClosedLoopSlot.kSlot0)
                .i(SLOT0_kI,ClosedLoopSlot.kSlot0)
                .d(SLOT0_kD,ClosedLoopSlot.kSlot0)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .outputRange((-1 * maxOutput),maxOutput)
                .pid(SLOT1_kP, SLOT1_kI, SLOT1_kD,ClosedLoopSlot.kSlot1) // slot 1, feed forward
                .feedForward
                  .kS(0.0,ClosedLoopSlot.kSlot1) // slot 1
                  .kV(SLOT1_kV, ClosedLoopSlot.kSlot1) // slot 1
                ;                        
      m_baseConfig_a.idleMode(IdleMode.kCoast)
                  //.smartCurrentLimit(Constants.LaunchConstants.MAX_CURRENT)
                  //.voltageCompensation(Constants.LaunchConstants.MAX_VOLTAGE)
                  ;

      //Update the motoro config to use PID
      //m_motor_a.setInverted(isRunning);
      m_motor_a.configure(m_baseConfig_a.inverted(true), ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
      //update config for follower
      m_baseConfig_b.closedLoop
                .p(SLOT0_kP,ClosedLoopSlot.kSlot0)
                .i(SLOT0_kI,ClosedLoopSlot.kSlot0)
                .d(SLOT0_kD,ClosedLoopSlot.kSlot0)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .outputRange((-1 * maxOutput),maxOutput)
                .pid(SLOT1_kP, SLOT1_kI, SLOT1_kD,ClosedLoopSlot.kSlot1) // slot 1, feed forward
                .feedForward
                  .kS(0.0,ClosedLoopSlot.kSlot1) // slot 1
                  .kV(SLOT1_kV, ClosedLoopSlot.kSlot1) // slot 1
                ;                        
      m_baseConfig_b.idleMode(IdleMode.kCoast)
                  ;
      m_baseConfig_b.follow(Constants.LaunchConstants.LAUNCH_MOTOR_ID_A,true);
      m_motor_b.configure(m_baseConfig_b, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  
      SmartDashboard.setDefaultNumber("Launch Target Velocity", defaultSetPointRPM);
      SmartDashboard.setDefaultBoolean("Launch Override Velocity", false);
      SmartDashboard.setDefaultBoolean("Launch Update PID", false);
      SmartDashboard.setDefaultBoolean("Launch PID Slot 0", true);
      SmartDashboard.putNumber("Launch P Gain", SLOT0_kP);
      SmartDashboard.putNumber("Launch I Gain", SLOT0_kI);
      SmartDashboard.putNumber("Launch D Gain", SLOT0_kD);
      SmartDashboard.putNumber("Launch V Gain", 0);
      SmartDashboard.putNumber("Launch IAccum", 0);
      SmartDashboard.putNumber("Launch Current Velocity",0);
  }

 public void stop()
  {
    isRunning = false;

    //set the current
    closedLoopController_a.setSetpoint(0, ControlType.kVelocity,ClosedLoopSlot.kSlot0);
    closedLoopController_a.setSetpoint(0, ControlType.kVelocity,ClosedLoopSlot.kSlot1);
    currentSetPointRPM = 0;
    //turn off motor
    m_motor_a.setVoltage(0);
    //turn off the control mode
  }

  /**
   * Run the launcher to a given velocity
   * @param targetVelocityRPM
   */
  public void runLauncher(double targetVelocityRPM)
  {
    if (!isRunning)
    {
      isRunning = true;
    }
    //Only update the launcher if the speed changes
    if (currentSetPointRPM != targetVelocityRPM)
    {
      currentSetPointRPM = targetVelocityRPM;
      SmartDashboard.putNumber("Launch Target Velocity", targetVelocityRPM);
      if (x_useSlot0)
        closedLoopController_a.setSetpoint(targetVelocityRPM, ControlType.kVelocity,ClosedLoopSlot.kSlot0);
      else
        closedLoopController_a.setSetpoint(targetVelocityRPM, ControlType.kVelocity,ClosedLoopSlot.kSlot1);
    }
  }
   
  /**
   * Runs the launcher to the default configured velocity
   */
  public void runLauncher() 
  {
    x_useTargetDistance = true;
    runLauncher(defaultSetPointRPM);
  }

  /**
   * Use turret estimated distance to target to adjust launch velocity
   * @param useTargetDistance
   */
  public void runLauncher(boolean useTargetDistance)
  {
    x_useTargetDistance = useTargetDistance;
    if (x_useTargetDistance)
       runLauncher(estimateVelocityForTargetDistance(RobotContainer.turretSubsystem.getDistanceToTarget()));
    else runLauncher(SmartDashboard.getNumber("Launch Target Velocity", 0));
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
    double tolerance = Constants.LaunchConstants.TOLERANCE; // RPM tolerance
    return Math.abs(currentSetPointRPM - getVelocity()) < tolerance;
  }

  /** 
   * Use some function to get the target velocity based on distance to target
   */
  private double estimateVelocityForTargetDistance(double targetDistanceMeters)
  {
    //Found 2500RPM works from 3.5-3.7
    double newTargetVelocity = Constants.LaunchConstants.MIN_SHOOTING_MIN_VELOCITY_RPM;
    //If we are further than a max distance just shoot full power
    if (targetDistanceMeters >= Constants.LaunchConstants.MAX_SHOOTING_DISTANCE)
    {
      newTargetVelocity = Constants.LaunchConstants.TARGET_VELOCITY_RPM;
    }
    else if (targetDistanceMeters >= 3.2)
    {
      //We know 3.2  - 3.7 at least works at 2500, can remove if we want to use equation for everything
      newTargetVelocity = 2500;
    }
    else
    {
      //first rough equation, best fit from a few values - https://mycurvefit.com/
      newTargetVelocity = 153.8462*targetDistanceMeters + 1930.769;

      //Assume 200 rpm for every .3 Meters
      //newTargetVelocity = ((3.5 - targetDistanceMeters ) / 0.3) * 200;
    }

    //If we are below a min velocity just use the min velocity
    if (newTargetVelocity < Constants.LaunchConstants.MIN_SHOOTING_MIN_VELOCITY_RPM) newTargetVelocity = Constants.LaunchConstants.MIN_SHOOTING_MIN_VELOCITY_RPM;

    SmartDashboard.putNumber("Launch Calc Velocity", newTargetVelocity);
    return newTargetVelocity;
  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
    if (isRunning)    
    {
      //run the launcher with either dashboard or calculated distance
      runLauncher(x_useTargetDistance);
    }
    boolean useSlot0 = SmartDashboard.getBoolean("Launch PID Slot 0", true);
    if (useSlot0 != x_useSlot0)
    {
      //Update dashboard values
      if (useSlot0)
      {
        SmartDashboard.putNumber("Launch P Gain", SLOT0_kP);
        SmartDashboard.putNumber("Launch I Gain", SLOT0_kI);
        SmartDashboard.putNumber("Launch D Gain", SLOT0_kD);
        SmartDashboard.putNumber("Launch V Gain", 0);
      }
      else
      {
        SmartDashboard.putNumber("Launch P Gain", SLOT1_kP);
        SmartDashboard.putNumber("Launch I Gain", SLOT1_kI);
        SmartDashboard.putNumber("Launch D Gain", SLOT1_kD);
        SmartDashboard.putNumber("Launch V Gain", SLOT1_kV);
      }
      x_useSlot0 = useSlot0;
    }

    if (SmartDashboard.getBoolean("Launch Update PID", false))
    {
      SmartDashboard.putBoolean("Launch Update PID", false);
            
      //Read PID values
      // read PID coefficients from SmartDashboard
      double p = SmartDashboard.getNumber("Launch P Gain", 0);
      double i = SmartDashboard.getNumber("Launch I Gain", 0);
      double d = SmartDashboard.getNumber("Launch D Gain", 0);
      double v = SmartDashboard.getNumber("Launch V Gain", 0);
            
      // if PID coefficients on SmartDashboard have changed, write new values to controller
      boolean updatePID = false;
      if (x_useSlot0)
      {
        if((p != SLOT0_kP)) { updatePID = true;  SLOT0_kP = p; }
        if((i != SLOT0_kI)) { updatePID = true;  SLOT0_kI = i; }
        if((d != SLOT0_kD)) { updatePID = true; SLOT0_kD = d; }
      }
      else
      {
        if((p != SLOT1_kP)) { updatePID = true;  SLOT1_kP = p; }
        if((i != SLOT1_kI)) { updatePID = true;  SLOT1_kI = i; }
        if((d != SLOT1_kD)) { updatePID = true; SLOT1_kD = d; }
        if((v != SLOT1_kV)) { updatePID = true; SLOT1_kV = v; }
      }

      if (updatePID)
      {
        //Update the PID on close loopController
        m_baseConfig_a.closedLoop
                    .p(SLOT0_kP,ClosedLoopSlot.kSlot0)
                    .i(SLOT0_kI,ClosedLoopSlot.kSlot0)
                    .d(SLOT0_kD,ClosedLoopSlot.kSlot0)
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    .outputRange((-1 * maxOutput),maxOutput) // set PID 
                    .pid(SLOT1_kP, SLOT1_kI, SLOT1_kD,ClosedLoopSlot.kSlot1) // slot 1, feed forward
                    .feedForward
                      .kS(0.0,ClosedLoopSlot.kSlot1) // slot 1
                      .kV(SLOT1_kV, ClosedLoopSlot.kSlot1) // slot 1
                    ;
         m_baseConfig_b.closedLoop
                    .p(SLOT0_kP,ClosedLoopSlot.kSlot0)
                    .i(SLOT0_kI,ClosedLoopSlot.kSlot0)
                    .d(SLOT0_kD,ClosedLoopSlot.kSlot0)
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    .outputRange((-1 * maxOutput),maxOutput) // set PID 
                    .pid(SLOT1_kP, SLOT1_kI, SLOT1_kD,ClosedLoopSlot.kSlot1) // slot 1, feed forward
                    .feedForward
                      .kS(0.0,ClosedLoopSlot.kSlot1) // slot 1
                      .kV(SLOT1_kV, ClosedLoopSlot.kSlot1) // slot 1
                    ;
        //Update the motoro config to use PID
        m_motor_a.configure(m_baseConfig_a, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        //m_baseConfig_b.follow(Constants.LaunchConstants.LAUNCH_MOTOR_ID_A,true);
        m_motor_b.configure(m_baseConfig_b, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
      } //end if updatePID
    } //end update PID
    
    SmartDashboard.putNumber("Launch IAccum", closedLoopController_a.getIAccum());
    SmartDashboard.putNumber("Launch Current Velocity",getVelocity());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
