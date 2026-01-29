package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LaunchSubsystem;
import frc.robot.subsystems.StageSubsystem;

public class Shoot extends Command {
    private final LaunchSubsystem launcher;
    private final StageSubsystem stager;
    private double startTime = 0;
    //private final BeamBreak beamBreak;

    public Shoot(LaunchSubsystem launchSubsystem, StageSubsystem stageSubsystem /* BeamBreak beamBreak*/) 
    {
        this.launcher = launchSubsystem;
        this.stager = stageSubsystem;
        //this.beamBreak = beamBreak;
        addRequirements(launchSubsystem, stageSubsystem /*, beamBreak */);
    }

    @Override
    public void initialize() {
        launcher.runLauncher();
        //start a timer, use until we get a fuel sensor
        startTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        launcher.runLauncher();
        if (launcher.atTargetVelocity()) {
            stager.runStage();
        } else {
            stager.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        launcher.stop();
        stager.stop();
    }

    @Override
    public boolean isFinished() {
        double delta = System.currentTimeMillis() - startTime;
        //return beamBreak.get() == false;

        //Run for 7 seconds
        return (delta > 7000);
    }

}
