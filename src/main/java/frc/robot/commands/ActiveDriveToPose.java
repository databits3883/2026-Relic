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
  private Pose2d firstGoalPose2d = Pose2d.kZero;
  private boolean isRed = true;
  private Transform2d poseError = Transform2d.kZero;

  private Timer loopTimer = new Timer();
  private boolean inAuto = true;
  private boolean atTolerance = false;
  private boolean finishedFirstStage = false;
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

    isRed = Robot.isRedAlliance;
    if (goalType == GoalType.Climber_Right) { if (isRed) goalType = GoalType.Climber_Red_Right; else goalType = GoalType.Climber_Blue_Right;}
    if (goalType == GoalType.Climber_Left) { if (isRed) goalType = GoalType.Climber_Red_Left; else goalType = GoalType.Climber_Blue_Left;}
    inAuto = inAutonomous;    

    rotationController.enableContinuousInput(-Math.PI, Math.PI);
    
    if(!(inAutonomous))
    {
      addRequirements(swerveSubsystem);
    }
    
    SmartDashboard.putData(positionController);
  }

  //Find best climb based on Y and alliance
  /*
  public ActiveDriveToPose(SwerveSubsystem swerveSubsystem, boolean inAutonomous)
  {
    isRed = Robot.isRedAlliance;
    Pose2d currentPose = null;
    drivetrain = swerveSubsystem;
    if (isRed) currentPose = Constants.DrivebaseConstants.INITITAL_RED_POSE; else currentPose = Constants.DrivebaseConstants.INITITAL_BLUE_POSE;
    if (swerveSubsystem != null)
    {
      currentPose = swerveSubsystem.getPose();
    }
    double currentY = currentPose.getY();
    if (isRed) 
    {
      if (currentY > Constants.Climber.RED_MID_CLIMBER_BAR) 
        goalType = ActiveDriveToPose.GoalType.Climber_Red_Right;
      else
        goalType = ActiveDriveToPose.GoalType.Climber_Red_Left;
    }
    else
    {
      if (currentY > Constants.Climber.BLUE_MID_CLIMBER_BAR) 
        goalType = ActiveDriveToPose.GoalType.Climber_Blue_Left;
      else
        goalType = ActiveDriveToPose.GoalType.Climber_Blue_Right;
    }
    inAuto = inAutonomous;    

    rotationController.enableContinuousInput(-Math.PI, Math.PI);
    
    if(!(inAutonomous))
    {
      addRequirements(swerveSubsystem);
    }
    
    SmartDashboard.putData(positionController);
  }
    */


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch (goalType)
    {
        case Climber_Blue_Left: goalPose2d =Constants.Climber.BLUE_LEFT_POSE; break;
        case Climber_Blue_Right: goalPose2d =Constants.Climber.BLUE_RIGHT_POSE; break;
        case Climber_Red_Left: goalPose2d =Constants.Climber.RED_LEFT_POSE; break;
        case Climber_Red_Right: goalPose2d =Constants.Climber.RED_RIGHT_POSE; break;
        default: goalPose2d = drivetrain.getPose();
    }
    //Set first goal pose to be 4 inches behind bar, after there then drive forward
    if (isRed) firstGoalPose2d = goalPose2d.transformBy(new Transform2d(Units.inchesToMeters(-1 * Constants.AutonConstants.WAY_POINT_BEHIND_BAR),0,new Rotation2d(0)));
    else  firstGoalPose2d = goalPose2d.transformBy(new Transform2d(Units.inchesToMeters(Constants.AutonConstants.WAY_POINT_BEHIND_BAR),0,new Rotation2d(0)));

    //Draw goal on field
    if (RobotContainer.DISPLAY_CLIMB_TARGET_POSE)
        RobotContainer.drivebase.getField().getObject("Climber Target Pose").setPose(goalPose2d);                                         

    atTolerance = false;
    finishedFirstStage = false;

    poseError = drivetrain.getPose().minus(firstGoalPose2d);
    drivetrain.goalPose2d = firstGoalPose2d;
    
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
    if (finishedFirstStage)
        poseError = currentPose.minus(goalPose2d);
    else    
        poseError = currentPose.minus(firstGoalPose2d);

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


  /**
   * Check if we are close enough
   * @return
   */
  public boolean atToleranceFromGoal()
  {
    double angleError = poseError.getRotation().getDegrees();

    double positionErrorMagnitude = poseError.getTranslation().getDistance(Translation2d.kZero);
    
    return (Math.abs(angleError) < 1.0) && positionErrorMagnitude < 0.035;    
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
    if (!finishedFirstStage && aligned)
    {
        //Mark that we reached the first goal, move on to the second
        finishedFirstStage = true;
        aligned = false;
        //update the goal on the drivetrain
        drivetrain.goalPose2d = goalPose2d;
    }

    //Use the aligned method only when in auto, otherwise joystick button keeps running this
    if (inAuto) 
        return aligned;     
    else 
        return false;
  }
}