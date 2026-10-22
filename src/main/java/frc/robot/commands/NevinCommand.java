package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LaunchSubsystem;
import frc.robot.subsystems.StageSubsystem;

public class NevinCommand extends Command {
    private final IntakeSubsystem intake;
    private final LaunchSubsystem launcher;
    private final StageSubsystem stager;
    public NevinCommand (IntakeSubsystem intakesub, LaunchSubsystem launchsub, StageSubsystem stage) {
        this.intake = intakesub;
        this.launcher = launchsub;
        this.stager = stage;
        addRequirements(launchsub, stage, intakesub);


    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
        intake.stopIntake();
        stager.stopStageSystem();
        launcher.stop();
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        super.execute();
        launcher.runLauncher(false);
        
        if (launcher.atTargetVelocity()) {
            stager.runSpindexer();
            stager.runOmnidexer();
            stager.runStage();
        } else {
            //stager.stopStageSystem();
        }
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
        System.out.println("NEVIN COMMAND WORKS");
        //set target launcher velocity
        SmartDashboard.putNumber("Launch Target Velocity", 1000);

        intake.runIntake();
        

    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return super.isFinished();
    }
    

}
