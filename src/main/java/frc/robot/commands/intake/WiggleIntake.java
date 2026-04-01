// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class WiggleIntake extends Command {
  private final IntakeSubsystem intake;
  private final double ABORT_TIME = Constants.Intake.FOUR_BAR_WIGGLE_TIMEOUT_SEC;
  private final double WIGGLE_POWER = Constants.Intake.FOUR_BAR_WIGGLE_POWER;
  private final Timer wiggle_alt_timer = new Timer();
  private boolean intakeUp=false;

  /** Creates a new WiggleIntake. */
  public WiggleIntake(IntakeSubsystem intakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intakeSubsystem;

    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeUp=true;
    //Stop the intake
    intake.stopIntake();

    //start a timer, We can stop after X seconds if it does not reach limit
    wiggle_alt_timer.restart();
    System.out.println("starting wiggle");

    execute();//make sure the intake is extending or retracting as it should
    
  }
    

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if (wiggle_alt_timer.hasElapsed(ABORT_TIME*2) && intakeUp==false){      
      wiggle_alt_timer.restart();
      intake.reverseFourBar();
      intakeUp=true;
    }
    if (wiggle_alt_timer.hasElapsed(ABORT_TIME) && intakeUp==true){
      wiggle_alt_timer.restart();
      intake.runFourBar();
      intakeUp=false;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopFourBar();
  }
    
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
