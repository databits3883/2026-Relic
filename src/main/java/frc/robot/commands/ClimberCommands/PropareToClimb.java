package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;

public class PropareToClimb extends Command {
    private final ClimberSubsystem climberSub;
    private final long ABORT_TIME = Constants.Climber.CLIMBER_TIMEOUT_SEC * 1000; /* in millis */
    private long startTime = 0;

    public PropareToClimb(ClimberSubsystem climberSubsystem) 
    {
        this.climberSub = climberSubsystem;

        addRequirements(climberSub);
    }

    @Override
    public void initialize() {
        climberSub.reverseClimber();
        //start a timer, We can stop after X seconds if it does not reach limit
        startTime = System.currentTimeMillis();
    }

    @Override
    public void execute() 
    {
        //Nothing to do here
    }

    @Override
    public void end(boolean interrupted) 
    {
        //Stop running if told to stop
        climberSub.stopClimber();
    }

    @Override
    public boolean isFinished() 
    {
        //Calculate the delta time in ms and we can abort after X milliseconds
        long delta = System.currentTimeMillis() - startTime;
        
        boolean atLimit = climberSub.isClimberForwardLimit();
        boolean overTime = (delta > ABORT_TIME);
        boolean finished = false;
        if (atLimit) finished = true;
        if (overTime) finished = true;
        
        //Stop if at limit or if we ran too long
        return finished;
    }

}

