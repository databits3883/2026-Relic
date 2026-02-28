// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ActiveDriveToPose;
import frc.robot.commands.ActiveDriveToPose.GoalType;
import frc.robot.commands.Outtake;
import frc.robot.commands.Shoot;
import frc.robot.commands.TurretAutoAim;
import frc.robot.commands.TurretManualAim;
import frc.robot.commands.ClimberCommands.Climb;
import frc.robot.commands.ClimberCommands.PrepareToClimb;
import frc.robot.commands.ClimberCommands.StowClimber;
import frc.robot.commands.intake.Deploy;
import frc.robot.commands.intake.Retract;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LaunchSubsystem;
import frc.robot.subsystems.StageSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public static final CommandJoystick driverJoystick = new CommandJoystick(0);
  private final       CommandJoystick copilotSNESController = new CommandJoystick(1);

  // The robot's subsystems and commands are defined here...
  public static final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/relic"));
  public static TurretSubsystem turretSubsystem  = new TurretSubsystem();
  public static StageSubsystem stageSubsystem  = new StageSubsystem();
  public static LaunchSubsystem launchSubsystem = new LaunchSubsystem();
  public static IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  public static ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  
  //debug/field/dashboard, etc
  public static boolean DISPLAY_VISION_TAGS = true;
  public static boolean DISPLAY_VISION_POSE = false;
  public static boolean DISPLAY_TARGET = true;
  public static boolean CAN_SHOOT = false;
  public static boolean ABOUT_TO_BE_ABLE_TO_SHOOT = false;

  // Establish a Sendable Chooser that will be able to be sent to the SmartDashboard, allowing selection of desired auto
  private final SendableChooser<Command> autoChooser;

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                            () -> driverJoystick.getY() * -1,
                                                            () -> driverJoystick.getX() * -1)
                                                        .withControllerRotationAxis(() -> driverJoystick.getTwist() * 0.65)
                                                        .deadband(OperatorConstants.DEADBAND)
                                                        .scaleTranslation(0.8)
                                                        .allianceRelativeControl(true);                                                            

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(() -> driverJoystick.getX() * -1,
                                                                                             () -> driverJoystick.getY() * -1)
                                                           .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                         () -> -driverJoystick.getY(),
                                                                         () -> -driverJoystick.getX())
                                                                    .withControllerRotationAxis(() -> driverJoystick.getRawAxis(
                                                                        2))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                  driverJoystick.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                  driverJoystick.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2))
                                                                               .headingWhile(true)
                                                                               .translationHeadingOffset(true)
                                                                               .translationHeadingOffset(Rotation2d.fromDegrees(
                                                                                   0));

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    
    //Create the NamedCommands that will be used in PathPlanner
    NamedCommands.registerCommand("AutoAlign R", new ActiveDriveToPose(drivebase, true,GoalType.Climber_Right));    
    NamedCommands.registerCommand("AutoAlign L", new ActiveDriveToPose(drivebase, true,GoalType.Climber_Left));    

    //Have the autoChooser pull in all PathPlanner autos as options
    autoChooser = AutoBuilder.buildAutoChooser();

    //Set the default auto (do nothing) 
    autoChooser.setDefaultOption("Do Nothing", Commands.none());

    //Add a simple auto option to have the robot drive forward for 1 second then stop
    autoChooser.addOption("Drive Forward 3sec", drivebase.driveForward().withTimeout(3));

    //Add a simple auto option to have the robot drive forward for 1 second then stop
    autoChooser.addOption("Deploy/Shoot 20sec", new ParallelCommandGroup(new Deploy(intakeSubsystem),new WaitCommand(1).andThen(new Shoot(launchSubsystem, stageSubsystem).withTimeout(20))));
    autoChooser.addOption("Shoot 20sec", new WaitCommand(1).andThen(new Shoot(launchSubsystem, stageSubsystem).withTimeout(20)));
    
    //Put the autoChooser on the SmartDashboard
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Straightens the robot on the field with intake forward depending on red/blue alliance , use if no vision targets
   **/
  private void straightenRobot()
  {
    if (Robot.isRedAlliance)
    {
      //Set the pose 180 degrees
      drivebase.resetOdometry(new Pose2d(drivebase.getPose().getTranslation(), Rotation2d.fromDegrees(180)));
    } 
    else
    {
      drivebase.resetOdometry(new Pose2d(drivebase.getPose().getTranslation(), new Rotation2d()));
    }
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    //Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    //Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    //Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    //Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    //Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleKeyboard);

    if (RobotBase.isSimulation())
    {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation())
    {
      Pose2d target = new Pose2d(new Translation2d(1, 4),
                                 Rotation2d.fromDegrees(90));
      //drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
      driveDirectAngleKeyboard.driveToPose(() -> target,
                                           new ProfiledPIDController(5,
                                                                     0,
                                                                     0,
                                                                     new Constraints(5, 2)),
                                           new ProfiledPIDController(5,
                                                                     0,
                                                                     0,
                                                                     new Constraints(Units.degreesToRadians(360),
                                                                                     Units.degreesToRadians(180))
                                           ));
      driverJoystick.button(13).onTrue(Commands.runOnce(drivebase::zeroGyro));

      driverJoystick.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
      driverJoystick.button(2).whileTrue(Commands.runEnd(() -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
                                                     () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));

    }
    if (DriverStation.isTest())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      driverJoystick.button(14).whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverJoystick.button(15).whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
    } 
    else
    {
      //straighten robot with the correct alliance
      driverJoystick.button(13).onTrue(Commands.runOnce(this::straightenRobot));
      //Death Spin, todo update button location
      driverJoystick.button(11).whileTrue(new RunCommand(() -> drivebase.setChassisSpeeds(new ChassisSpeeds(0, 0, 12)), drivebase));
      
      driverJoystick.button(15).whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      
      //Start or stop aiming at target, call the TurretAim to toggle on/off      
      driverJoystick.button(14).onTrue(new TurretAutoAim());
      //manual aim override button
      driverJoystick.button(12).whileTrue(new TurretManualAim());
      driverJoystick.povUp().onTrue(Commands.runOnce(() -> { turretSubsystem.setManualAimTarget(0);}));
      driverJoystick.povDown().onTrue(Commands.runOnce(() -> { turretSubsystem.setManualAimTarget(180);}));
      driverJoystick.povLeft().onTrue(Commands.runOnce(() -> { turretSubsystem.setManualAimTarget(90);}));
      driverJoystick.povRight().onTrue(Commands.runOnce(() -> { turretSubsystem.setManualAimTarget(270);}));

      //Shoot while button is held, auto distance
      driverJoystick.button(1).whileTrue(new Shoot(launchSubsystem, stageSubsystem, false));
      copilotSNESController.button(5).whileTrue(new Shoot(launchSubsystem, stageSubsystem, false));
      //Shoot with manual velocity control
      driverJoystick.button(5).whileTrue(new Shoot(launchSubsystem, stageSubsystem, true));

      //Outake while button is held
      driverJoystick.button(2).whileTrue(new Outtake(launchSubsystem, stageSubsystem));

      //Intake deploy/retract
      driverJoystick.button(4).onTrue(new Deploy(intakeSubsystem));
      copilotSNESController.axisLessThan(0,-0.5).onTrue(new Deploy(intakeSubsystem));
      driverJoystick.button(3).onTrue(new Retract(intakeSubsystem));
      copilotSNESController.axisGreaterThan(0,0.5).onTrue(new Retract(intakeSubsystem));
      //Run intake while this button is off and stop when button is on
      copilotSNESController.button(3).onTrue(Commands.runOnce(intakeSubsystem::toggleIntake));

      //Climber
      //climber stow/prepare
      driverJoystick.button(7).onTrue(new Retract(intakeSubsystem).andThen(new PrepareToClimb(climberSubsystem)));
      copilotSNESController.button(9).onTrue(new Retract(intakeSubsystem).andThen(new PrepareToClimb(climberSubsystem)));
      //climb
      driverJoystick.button(8).onTrue(new Climb(climberSubsystem));
      copilotSNESController.button(10).onTrue(new Climb(climberSubsystem));
      //Stow
      driverJoystick.button(9).onTrue(new StowClimber(climberSubsystem));
      copilotSNESController.button(4).onTrue(new StowClimber(climberSubsystem));
      
      //Manually Aim when on
      //copilotBoxController.button(10).whileTrue(new TurretManualAim());

      //Test later
      driverJoystick.button(6).whileTrue(new SequentialCommandGroup(new ActiveDriveToPose(drivebase, true, GoalType.Climber_Right),new Climb(climberSubsystem)));
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // Pass in the selected auto from the SmartDashboard as our desired autnomous commmand 
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}