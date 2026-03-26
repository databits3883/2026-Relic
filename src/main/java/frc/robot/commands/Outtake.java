package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LaunchSubsystem;
import frc.robot.subsystems.StageSubsystem;

public class Outtake extends Command {
    private final LaunchSubsystem launcher;
    private final StageSubsystem stager;
    private final IntakeSubsystem intake;
    private final double maxLauncherSpeed = Constants.LaunchConstants.TARGET_VELOCITY_RPM;
    private final double maxStageSpeed = Constants.StageConstants.TARGET_VELOCITY_RPS;
    private final double maxSpindexerPower = Constants.StageConstants.SPINDEXER_MOTOR_POWER;
    private final double maxOmnidexerPower = Constants.StageConstants.OMNIDEXER_MOTOR_POWER;

    public Outtake(LaunchSubsystem launchSubsystem, StageSubsystem stageSubsystem, IntakeSubsystem intakeSubsystem) 
    {
        this.launcher = launchSubsystem;
        this.stager = stageSubsystem;
        this.intake = intakeSubsystem;
        
        addRequirements(launchSubsystem, stageSubsystem, intakeSubsystem);
    }

    @Override
    public void initialize() 
    {
        launcher.runLauncher(-1 * maxLauncherSpeed);
        stager.runStageVelocity(-1 * maxStageSpeed);
        stager.runSpindexer(-1 * maxSpindexerPower);
        stager.runOmnidexer(-1 * maxOmnidexerPower);        
        intake.runPowerIntake(-1 * Constants.Intake.INTAKE_MOTOR_MAX_POWER);
    }

    @Override
    public void execute() 
    {
        //Run until completed
    }

    @Override
    public void end(boolean interrupted) 
    {
        launcher.stop();
        stager.stopStageSystem();
        intake.stopIntake();
    }

    @Override
    public boolean isFinished() 
    {
        //Run until user released button, which will call end
        return false;
    }
}