// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.AutonConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ActiveDriveToPose extends Command {

  public enum GoalType {
    Climber_Right,
    Climber_Left
  }

  private GoalType goalType = GoalType.Climber_Right;

  private SwerveSubsystem drivetrain;
  private Pose2d goalPose2d = Pose2d.kZero;
  private boolean isRed = true;
  private Transform2d poseError = Transform2d.kZero;

  private Timer loopTimer = new Timer();
  private boolean inAuto = true;
  private boolean atTolerance = false;
  private Timer timeAtTolerance = new Timer();

  private PIDController positionController = new PIDController(AutonConstants.positionKP, AutonConstants.positionKI, AutonConstants.positionKD);
//  private TrapezoidProfile.State previousPositionState = new State(0, 0);
//  private TrapezoidProfile positionTrapezoidProfile = new TrapezoidProfile(AutonConstants.positionPIDConstraints);

  private PIDController positionXController = new PIDController(AutonConstants.positionKP, AutonConstants.positionKI, AutonConstants.positionKD);
  private TrapezoidProfile.State previousPositionXState = new State(0, 0);
  private TrapezoidProfile positionXTrapezoidProfile = new TrapezoidProfile(AutonConstants.positionPIDConstraints);

  private PIDController positionYController = new PIDController(AutonConstants.positionKP, AutonConstants.positionKI, AutonConstants.positionKD);
  private TrapezoidProfile.State previousPositionYState = new State(0, 0);
  private TrapezoidProfile positionYTrapezoidProfile = new TrapezoidProfile(AutonConstants.positionPIDConstraints);
  
  private PIDController rotationController = new PIDController(AutonConstants.rotationKP, AutonConstants.rotationKI, AutonConstants.rotationKD);
//  private TrapezoidProfile.State previousRotationState = new State(0, 0);
//  private TrapezoidProfile positionRotationProfile = new TrapezoidProfile(AutonConstants.rotationPIDConstraints);

  /** Creates a new ActiveDriveToGoalPose. */
  public ActiveDriveToPose(SwerveSubsystem swerveSubsystem, boolean inAutonomous, GoalType goal) 
  {
    drivetrain = swerveSubsystem;

    goalType = goal;

    isRed = Robot.isRedAlliance;
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
  public void initialize() {
    if(goalType == GoalType.Climber_Left)
    {
      goalPose2d = (isRed ? Constants.Climber.RED_LEFT_POSE: Constants.Climber.BLUE_LEFT_POSE);
    }
    else if (goalType == GoalType.Climber_Right){
      goalPose2d = (isRed ? Constants.Climber.RED_RIGHT_POSE: Constants.Climber.BLUE_RIGHT_POSE);
    }
    else
    {
      goalPose2d = drivetrain.getPose();
    }

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

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    Pose2d currentPose = drivetrain.getPose();
    poseError = currentPose.minus(goalPose2d);
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
  }


  public boolean atToleranceFromGoal()
  {
    double angleError = poseError.getRotation().getDegrees();
    double positionErrorMagnitude = poseError.getTranslation().getDistance(Translation2d.kZero);
    
    return (Math.abs(angleError) < 1.0) && positionErrorMagnitude < 0.035;    
  }

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