package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LaunchSubsystem;
import frc.robot.subsystems.StageSubsystem;

public class Outtake extends Command {
    private final LaunchSubsystem launcher;
    private final StageSubsystem stager;
    private final double maxLauncherSpeed = Constants.LaunchConstants.TARGET_VELOCITY_RPS;
    private final double maxStageSpeed = Constants.StageConstants.TARGET_VELOCITY_RPS;
    private final double maxSpindexerPower = Constants.StageConstants.SPINDEXER_MOTOR_POWER;
    private final double maxOmnidexerPower = Constants.StageConstants.OMNIDEXER_MOTOR_POWER;

    public Outtake(LaunchSubsystem launchSubsystem, StageSubsystem stageSubsystem) 
    {
        this.launcher = launchSubsystem;
        this.stager = stageSubsystem;
        
        addRequirements(launchSubsystem, stageSubsystem);
    }

    @Override
    public void initialize() 
    {
        launcher.runLauncher(-1 * maxLauncherSpeed);
        stager.runStage(-1 * maxStageSpeed);
        stager.runSpindexer(-1 * maxSpindexerPower);
        stager.runOmnidexer(-1 * maxOmnidexerPower);
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
        stager.stopSpindexer();
        stager.stopOmnidexer();
        stager.stop();
    }

    @Override
    public boolean isFinished() 
    {
        //Run until user released button, which will call end
        return false;
    }
}