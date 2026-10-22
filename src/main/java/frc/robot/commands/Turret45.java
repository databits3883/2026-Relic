package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LaunchSubsystem;
import frc.robot.subsystems.StageSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.RobotContainer;
import frc.robot.commands.TurretManualAim;

public class Turret45 extends Command {
    private final LaunchSubsystem launcher;
    private final TurretSubsystem turret;
    private final StageSubsystem stage;
    private final IntakeSubsystem intake;
    private double turretAngle = 0;

    public Turret45 (IntakeSubsystem intakeSub, StageSubsystem stageSub, TurretSubsystem turretSub, LaunchSubsystem launcherSub) {
        
        this.intake = intakeSub;
        this.stage = stageSub;
        this.turret = turretSub;
        this.launcher = launcherSub;
        
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
        System.out.println("Initializing Turret45");
        intake.runIntake();
        launcher.runLauncher();
        stage.runStage();
        turret.enableManuallyAim();
        turret.setManualAimTarget(turretAngle);
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        super.execute();
        /*
        check timer, every 1 second
        turretAngle + 45 MOD 360;
        TurretSubsystem.setManualAngle(turrentAngle)
        */
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
        intake.stopIntake();
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return super.isFinished();
    }

    

}
