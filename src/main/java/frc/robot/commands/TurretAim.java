// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurretAim  extends Command {
  private double targetAngle = 0;
  private final double DEADBAND = 4; //degrees in deadband

  /** Creates a new runSpinner. */
  public TurretAim (Optional<Pose3d> tagToAim, SwerveSubsystem swerveSubsystem) 
  {
      System.out.println("TurretAim:const:about to run constructor");
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.turretSubsystem);
    //TODO: use tag vs robots current pose
    Pose2d currentRobotPose = swerveSubsystem.getPose();

    //Get the april tag pose passed in, default to robot if not set
    Pose2d targetPose = currentRobotPose;
    if (tagToAim.isPresent()) targetPose = tagToAim.get().toPose2d();
    //Get the rotation Between robot and target
    Pose2d relativePose = currentRobotPose.relativeTo(targetPose);
    double relativeRotationDeg = relativePose.getRotation().getDegrees();
    //Get current robotRotation
    double robotHeadingDeg = swerveSubsystem.getHeading().getDegrees();
    //Need to know what to do with these angles, add heading?  Subtract heading?

    SmartDashboard.putNumber("Turret targetX", targetPose.getX());
    SmartDashboard.putNumber("Turret targetY", targetPose.getY());
    SmartDashboard.putNumber("Turret relativeRot", relativeRotationDeg);
    SmartDashboard.putNumber("Turret robotX", currentRobotPose.getX());
    SmartDashboard.putNumber("Turret robotY", currentRobotPose.getY());
    SmartDashboard.putNumber("Turret robotRot", robotHeadingDeg);

    //Right now set target to just the angle to tag
    targetAngle = relativeRotationDeg;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      System.out.println("TurretAim:init:about to run turret to " + targetAngle);
      double currentAngle = RobotContainer.turretSubsystem.getCurrentTurretAngle();
      double deltaAngle = Math.abs(currentAngle - targetAngle);

      //Only turn turret if greater than 5 degrees, 
      //TODO change this
      if (deltaAngle > DEADBAND) 
      {
        RobotContainer.turretSubsystem.setTurretSetPoint(targetAngle);
      }
      else end(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //TODO get updated robot pose
    //TODO update target angle setpoint
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted)
    {
      //System.out.println("RunSpinnner:end:interrupted was called!");
      //We do not need to do anything special if this gets interrupted early
    }
    System.out.println("TurretAim:end:about to stop motor");
    RobotContainer.turretSubsystem.stop();
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    double currentAngle = RobotContainer.turretSubsystem.getCurrentTurretAngle();
    double currentRots = Units.degreesToRotations(currentAngle);
    double targetRots = Units.degreesToRotations(targetAngle);
    double deadbandRots = Units.degreesToRotations(DEADBAND);
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