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
  static int ANGLE_OFFSET_DEFAULT = 10; //default degrees °

  int currentAngle = 0;
  int angleCount = 0;
  boolean switchFound = false;
  TurretSubsystem turret;

  /**
   * Try to align using the default of +- 10°
   */
  public TurretAlign()
  {
    this(ANGLE_OFFSET_DEFAULT);
  }

  /**
   * Try to align the turret by rotating X° in each direction to find switch
   * @param angleToStartAt
   */
  public TurretAlign(int angleToStartAt) 
  {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.turretSubsystem);
    turret = RobotContainer.turretSubsystem;
    currentAngle = angleToStartAt;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //System.out.println("Toggle Manual Aim Command init()");
    turret.enableManuallyAim();
    //Subtract the starting angle from zero and mod 360 to get a positive 
    currentAngle = (-1 * currentAngle + 360) % 360;
    turret.setManualAimTarget(currentAngle);
    angleCount = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {    
    //If we are close enough to the target check if we are on switch
    if (((int)turret.getCurrentAngle()) - currentAngle  <= 1)
    {
      if (!turret.isOnAlignSwitch())
      {
        currentAngle++;  //increase one degree
        //Loop around back to zero if 360 or higher
        if (currentAngle >= 360) currentAngle = currentAngle % 360;
        turret.setManualAimTarget(currentAngle);
        angleCount++;
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
    //If the turret rotated 360 and still did not see if, end but do not zero
    if (angleCount >= 360) end(true);

    //Otherwise continue to rotate turret
    return false;
  } 
}