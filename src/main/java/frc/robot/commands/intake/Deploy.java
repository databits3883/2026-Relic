package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class Deploy extends Command {
    private final IntakeSubsystem intake;
    private final long ABORT_TIME = Constants.Intake.FOUR_BAR_TIMEOUT_SEC * 1000; /* in millis */
    private long startTime = 0;

    public Deploy(IntakeSubsystem intakeSubsystem) 
    {
        this.intake = intakeSubsystem;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        intake.runFourBar();
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
        intake.stopFourBar();
    }

    @Override
    public boolean isFinished() 
    {
        //Calculate the delta time in ms and we can abort after X milliseconds
        long delta = System.currentTimeMillis() - startTime;
        
        boolean atLimit = intake.isFourBarForwardLimit();
        boolean overTime = (delta > ABORT_TIME);
        boolean finished = false;
        if (atLimit) finished = true;
        if (overTime) finished = true;
        
        //Stop if at limit or if we ran too long
        return finished;
    }

}
