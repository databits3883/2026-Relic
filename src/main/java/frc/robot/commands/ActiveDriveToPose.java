// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutonConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class ActiveDriveToPose extends Command {

  public enum GoalType {
    Climber_Red_Right,
    Climber_Red_Left,
    Climber_Blue_Right,
    Climber_Blue_Left,
    Climber_Right,
    Climber_Left
  }

  private GoalType goalType = GoalType.Climber_Red_Right;

  private SwerveSubsystem drivetrain;
  private Pose2d goalPose2d = Pose2d.kZero;
  private boolean isRed = true;
  private Transform2d poseError = Transform2d.kZero;

  private Timer loopTimer = new Timer();
  private boolean inAuto = true;
  private boolean atTolerance = false;
  private boolean errorFinish = false;
  private Timer timeAtTolerance = new Timer();

  private PIDController positionController = new PIDController(AutonConstants.positionKP, AutonConstants.positionKI, AutonConstants.positionKD);

  private PIDController positionXController = new PIDController(AutonConstants.positionKP, AutonConstants.positionKI, AutonConstants.positionKD);
  private TrapezoidProfile.State previousPositionXState = new State(0, 0);
  private TrapezoidProfile positionXTrapezoidProfile = new TrapezoidProfile(AutonConstants.positionPIDConstraints);

  private PIDController positionYController = new PIDController(AutonConstants.positionKP, AutonConstants.positionKI, AutonConstants.positionKD);
  private TrapezoidProfile.State previousPositionYState = new State(0, 0);
  private TrapezoidProfile positionYTrapezoidProfile = new TrapezoidProfile(AutonConstants.positionPIDConstraints);
  
  private PIDController rotationController = new PIDController(AutonConstants.rotationKP, AutonConstants.rotationKI, AutonConstants.rotationKD);

  /** Creates a new ActiveDriveToGoalPose. */
  public ActiveDriveToPose(SwerveSubsystem swerveSubsystem, boolean inAutonomous, GoalType goal) 
  {
    drivetrain = swerveSubsystem;
    goalType = goal;
    errorFinish = false;   

    inAuto = inAutonomous;    

    rotationController.enableContinuousInput(-Math.PI, Math.PI);
    
    if(!(inAutonomous))
    {
      addRequirements(swerveSubsystem);
    }
    
    SmartDashboard.putData(positionController);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    isRed = Robot.isRedAlliance;
    
    if (goalType == GoalType.Climber_Right) { if (isRed) goalType = GoalType.Climber_Red_Right; else goalType = GoalType.Climber_Blue_Right;}
    if (goalType == GoalType.Climber_Left) { if (isRed) goalType = GoalType.Climber_Red_Left; else goalType = GoalType.Climber_Blue_Left;}
    if (goalType == GoalType.Climber_Blue_Left) goalPose2d =Constants.Climber.BLUE_LEFT_POSE;
    else if (goalType == GoalType.Climber_Blue_Right) goalPose2d =Constants.Climber.BLUE_RIGHT_POSE; 
    else if (goalType == GoalType.Climber_Red_Left) goalPose2d =Constants.Climber.RED_LEFT_POSE; 
    else if (goalType == GoalType.Climber_Red_Right) goalPose2d =Constants.Climber.RED_RIGHT_POSE; 
    else errorFinish = true; /*No correct goal passed, return error */

    if (!errorFinish)
    {
        atTolerance = false;

        poseError = drivetrain.getPose().minus(goalPose2d);
        drivetrain.goalPose2d = goalPose2d;
        
        ChassisSpeeds currentSpeeds = drivetrain.getRobotVelocity();

        previousPositionXState.position = poseError.getX();
        previousPositionXState.velocity = -currentSpeeds.vxMetersPerSecond;//might need to be negative

        previousPositionYState.position = poseError.getY();
        previousPositionYState.velocity = -currentSpeeds.vyMetersPerSecond;//might need to be negative

        loopTimer.restart();
    }
    end(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    //end early on any error
    if (errorFinish) end(false);

    Pose2d currentPose = drivetrain.getPose();
    poseError = currentPose.minus(goalPose2d);

    //Draw goal on field
    if (RobotContainer.DISPLAY_CLIMB_TARGET_POSE)
    {
      RobotContainer.drivebase.getField().getObject("Climber Target Pose").setPose(goalPose2d);       
    }

    Translation2d translationError = poseError.getTranslation();
    ChassisSpeeds currentSpeeds = drivetrain.getRobotVelocity();

    TrapezoidProfile.State currentPositionXState = new State(translationError.getX(), -currentSpeeds.vxMetersPerSecond);
    TrapezoidProfile.State currentPositionYState = new State(translationError.getY(), -currentSpeeds.vyMetersPerSecond);

    previousPositionXState = positionXTrapezoidProfile.calculate(loopTimer.get(), currentPositionXState, new State(0,0));
    double positionXPIDOutput = positionXController.calculate(previousPositionXState.position, 0);

    previousPositionYState = positionYTrapezoidProfile.calculate(loopTimer.get(), currentPositionYState, new State(0,0));
    double positionYPIDOutput = positionYController.calculate(previousPositionYState.position, 0);

    double rotationPIDOutput = rotationController.calculate(poseError.getRotation().getRadians(), 0);
    
    ChassisSpeeds rrSpeeds = new ChassisSpeeds(positionXPIDOutput,positionYPIDOutput, rotationPIDOutput);
    drivetrain.setChassisSpeeds(rrSpeeds);
    
    loopTimer.restart();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    drivetrain.setChassisSpeeds(new ChassisSpeeds(0,0,0));
    //remove item
    if ((RobotContainer.DISPLAY_CLIMB_TARGET_POSE))
        RobotContainer.drivebase.getField().getObject("Climber Target Pose").setPose(new Pose2d());      
  }


  /**
   * Check if we are close enough
   * @return
   */
  public boolean atToleranceFromGoal()
  {
    double angleError = poseError.getRotation().getDegrees();

    double positionErrorMagnitude = poseError.getTranslation().getDistance(Translation2d.kZero);
    
    return (Math.abs(angleError) < 1.0) && positionErrorMagnitude < 0.050;    
  }

  /**
   * Are we at goal and in position to climb
   * @return
   */
  public boolean readyToClimb()
  {
    boolean nowAtTolerance = atToleranceFromGoal();
    if(!atTolerance && nowAtTolerance){
      timeAtTolerance.restart();
    }
    else if (!nowAtTolerance){
      timeAtTolerance.reset();
      timeAtTolerance.stop();
    }
    atTolerance = nowAtTolerance;

    return timeAtTolerance.hasElapsed(0.1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {  
    boolean aligned = readyToClimb();

    //Use the aligned method only when in auto, otherwise joystick button keeps running this
    if (inAuto) 
        return aligned;     
    else 
        return false;
  }
}