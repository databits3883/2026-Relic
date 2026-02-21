package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Climber;
import frc.robot.subsystems.ClimberSubsystem;

public class StowClimber extends Command {
    private final ClimberSubsystem climberSub;
    private final long ABORT_TIME = Constants.Climber.CLIMBER_TIMEOUT_SEC * 1000; /* in millis */
    private long startTime = 0;
    private long deltaRunAgain = 0;

    public StowClimber(ClimberSubsystem climberSubsystem) 
    {
        this.climberSub = climberSubsystem;

        addRequirements(climberSub);
    }
    public StowClimber(ClimberSubsystem climberSubsystem, int secondsToFireAgain) 
    {
        this(climberSubsystem);
        deltaRunAgain = 1000*secondsToFireAgain;
    }

    @Override
    public void initialize() {
        if (deltaRunAgain <= 0)
            climberSub.runClimber(Constants.Climber.SLOW_REVERSE_SPEED);
        //start a timer, We can stop after X seconds if it does not reach limit
        startTime = System.currentTimeMillis();
    }

    @Override
    public void execute() 
    {
        //check if we are running every x seconds
        if (deltaRunAgain > 0)
        {
            long delta = System.currentTimeMillis() - startTime;
            if (delta >= deltaRunAgain)
            {
                startTime = System.currentTimeMillis();   
                climberSub.runClimber(Constants.Climber.SLOW_REVERSE_SPEED);         
            }
        }
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

        //Check the soft limit
        boolean atLimit = climberSub.isClimberReverseLimit();        
        boolean overTime = (delta > ABORT_TIME);
        boolean isStalled = climberSub.isStalled();
        if (deltaRunAgain > 0) isStalled = false;
        
        boolean finished = false;
        if (atLimit) finished = true;
        if (overTime) finished = true;
        //if (isStalled) finished = true;
        
        //Stop if at limit or if we ran too long
        return finished;
    }

}

