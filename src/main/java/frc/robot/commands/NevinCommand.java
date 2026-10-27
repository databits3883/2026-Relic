package frc.robot.commands;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LaunchSubsystem;
import frc.robot.subsystems.StageSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class NevinCommand extends Command {
    private final IntakeSubsystem intake;
    private final LaunchSubsystem launcher;
    private final StageSubsystem stager;
    private final TurretSubsystem turret;
    private final Timer myTimer;
    private int angle;
    public NevinCommand (IntakeSubsystem intakesub, LaunchSubsystem launchsub, StageSubsystem stage, TurretSubsystem turretSub) {
        this.intake = intakesub;
        this.launcher = launchsub;
        this.stager = stage;
        this.turret = turretSub;
        addRequirements(launchsub, stage, intakesub,turretSub);
        myTimer = new Timer();
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
        intake.stopIntake();
        stager.stopStageSystem();
        launcher.stop();
        turret.disableManuallyAim();
        myTimer.stop();
        System.out.println("NEVIN COMMAND FINISHED");
        System.out.println("");
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
        if (myTimer.hasElapsed(2)) 
        {
            angle += 46;
            while(angle < 0){
                angle+=360;                
            }
            if (angle >= 360){
                angle =  angle % 360;
            }
            turret.setManualAimTarget(angle);
            System.out.println("angle updated to: "+angle);
            myTimer.reset();
        }
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
        System.out.println("NEVIN COMMAND WORKS");
        //set target launcher velocity
        SmartDashboard.putNumber("Launch Target Velocity", 1000);
        
        // enables manual aim and sets a setpoint 
        turret.enableManuallyAim();
        angle = 45;
        turret.setManualAimTarget(angle);
                        
        // runs intake
        intake.runIntake();

        // timer manage

        myTimer.reset();
        myTimer.start();
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return super.isFinished();
    }
    

}
