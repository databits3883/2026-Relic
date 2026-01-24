// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurretAim  extends Command {
  double targetAngle = 0;
  PhotonCamera x_camera = null;

  /** Creates a new runSpinner. */
  public TurretAim (PhotonCamera camera) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.turretSubsystem);
    x_camera = camera;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("TurretAim:init:about to run turret to " + targetAngle);
      PhotonPipelineResult result = null;
      double target = 0;
      System.out.println("TurretAim:const:checking camera");
      if (x_camera != null) 
      {
        System.out.println("TurretAim:const:camera!=null");
        List<PhotonPipelineResult>  allresults = x_camera.getAllUnreadResults();
        if ((allresults!= null) && (allresults.size() > 0)) result = allresults.get(0);
        if (result != null)
        {
            System.out.println("TurretAim:const:result!=null");
            boolean hasTargets = result.hasTargets();
            SmartDashboard.putBoolean("Turret Camera hasTargets",hasTargets);
            System.out.println("TurretAim:const:hasTargets="+hasTargets);

            PhotonTrackedTarget photonTarget = null;
            if (result.hasTargets()) photonTarget = result.getBestTarget();
            if (photonTarget != null)
            {
              SmartDashboard.putNumber("Turret Camera bestTarget",photonTarget.fiducialId);
              System.out.println("TurretAim:const:bestTarget="+photonTarget.fiducialId);
              double getYawOfTarget = photonTarget.getYaw();
              //assume this is in degrees
              System.out.println("TurretAim:const:getYawOfTarget="+getYawOfTarget);
              //set as new target
              target = getYawOfTarget;
            }
        }
      }
      if (target != 0) 
      {
        targetAngle = target;
        RobotContainer.turretSubsystem.setTurretSetPoint(targetAngle);
      }
      else end(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted)
    {
      //System.out.println("RunSpinnner:end:interrupted was called!");
      //We do not need to do anything special if this gets interrupted early
    }
    System.out.println("TurretAim:end:about to stop motor");
    //RobotContainer.armSubsystem.stop();
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    int deadband = 4; //degrees
    double currentAngle = RobotContainer.turretSubsystem.getCurrentTurretAngle();
    double currentRots = Units.degreesToRotations(currentAngle);
    double targetRots = Units.degreesToRotations(targetAngle);
    double deadbandRots = Units.degreesToRotations(deadband);
    //If we get inside the deadband stop
    if (Math.abs(currentRots - targetRots) <= deadbandRots)
    {
      RobotContainer.turretSubsystem.stop();
      return true;
    } 
    else
      return false;
  }
}