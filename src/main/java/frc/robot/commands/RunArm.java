// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunArm  extends Command {
  int targetAngle = 0;
  /** Creates a new runSpinner. */
  public RunArm (int m_targetAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.armSubsystem);
    targetAngle = m_targetAngle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("RunArm:init:about to stop arm");
    RobotContainer.armSubsystem.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted)
    {
      //System.out.println("RunSpinnner:end:interrupted was called!");
      //We do not need to do anything special if this gets interrupted early
    }
    System.out.println("RunArm:end:about to stop motor");
    RobotContainer.armSubsystem.stop();
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}