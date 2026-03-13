// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.TurretSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurretAlign  extends Command 
{
  static int MAX_ANGLE_DEFAULT = 10; //default degrees °
  int maxAngleToCheck = 0;
  int currentAngle = 0;
  boolean switchFound = false;
  TurretSubsystem turret;

  /**
   * Try to align using the default of +- 10°
   */
  public TurretAlign()
  {
    this(MAX_ANGLE_DEFAULT);
  }

  /**
   * Try to align the turret by rotating X° in each direction to find switch
   * @param maxAngleToRotate
   */
  public TurretAlign(int maxAngleToRotate) 
  {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.turretSubsystem);
    turret = RobotContainer.turretSubsystem;

    maxAngleToCheck = maxAngleToRotate;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //System.out.println("Toggle Manual Aim Command init()");
    turret.enableManuallyAim();
    currentAngle = -1 * maxAngleToCheck;
    turret.setManualAimTarget(currentAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {    
    if (((int)turret.getCurrentAngle()) == currentAngle)
    {
      if (!turret.isOnAlignSwitch())
      {
        currentAngle++;  //increase one degree        
      }
    }
    //wait to get to set point
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    if (!interrupted)
    {
      //We found the correct zero
      turret.zeroEncoder();
    }
    turret.disableManuallyAim();
  }

  // Imediate end this command
  @Override
  public boolean isFinished() 
  {
    //If the alignment switch is on, we are at the "zero"
    if (turret.isOnAlignSwitch()) return true;
    return false;
  } 
}