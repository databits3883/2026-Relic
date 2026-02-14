package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LaunchSubsystem;
import frc.robot.subsystems.StageSubsystem;

public class Shoot extends Command {
    private final LaunchSubsystem launcher;
    private final StageSubsystem stager;

    public Shoot(LaunchSubsystem launchSubsystem, StageSubsystem stageSubsystem) 
    {
        this.launcher = launchSubsystem;
        this.stager = stageSubsystem;
        
        addRequirements(launchSubsystem, stageSubsystem);
    }

    @Override
    public void initialize() {
        launcher.runLauncher();
    }

    @Override
    public void execute() {
        launcher.runLauncher();
        if (launcher.atTargetVelocity()) {
            stager.runSpindexer();
            stager.runOmnidexer();
            stager.runStage();
        } else {
            stager.stopStageSystem();
        }
    }

    @Override
    public void end(boolean interrupted) {
        launcher.stop();
        stager.stopStageSystem();
    }

    @Override
    public boolean isFinished() {
        //Run until user released button, which will call end
        return false;
    }
}