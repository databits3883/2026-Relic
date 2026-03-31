// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurretManualAim  extends Command 
{
  private double setPointAngle;
  private double launchSpeedOverride;
  private boolean overrideLaunchSpeed = false;

  /** Creates a new turretAim toggle command */
  public TurretManualAim() 
  {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.turretSubsystem);
    //get current angle
    setPointAngle = RobotContainer.turretSubsystem.getCurrentAngle();
  }
  public TurretManualAim(double angle, double launchSpeed)
  {
    this();
    setPointAngle = angle;
    overrideLaunchSpeed = true;
    launchSpeedOverride = launchSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //System.out.println("Toggle Manual Aim Command init()");
    RobotContainer.turretSubsystem.enableManuallyAim();
    RobotContainer.turretSubsystem.setManualAimTarget(setPointAngle);
    if (overrideLaunchSpeed)
    {
      RobotContainer.launchSubsystem.forceManualVelocity(true, launchSpeedOverride);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
      //System.out.println("Toggle Manual Aim Command execute()");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    //disable the forced launch speed
    RobotContainer.launchSubsystem.forceManualVelocity(false, 0);
    RobotContainer.turretSubsystem.disableManuallyAim();
    //System.out.println("Toggle Manual Aim Command end()");
  }

  // Imediate end this command
  @Override
  public boolean isFinished() 
  {
    //Run until the user releases the button
    return false;
  } 
}