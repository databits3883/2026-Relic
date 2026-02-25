// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot
{

  private static Robot   instance;
  private        Command m_autonomousCommand;

  private static final AprilTagFields fieldToUse = AprilTagFields.k2026RebuiltWelded;
  public static final AprilTagFieldLayout aprilTagFieldLayout_AllTags = AprilTagFieldLayout.loadField(fieldToUse);
  public static boolean isRedAlliance = false;
  private boolean hasRunAuto = false;

  private RobotContainer m_robotContainer;
  public char AUTO_WINNER_CODE = 'U';

  private Timer disabledTimer;

  public Robot()
  {
    instance = this;
  }

  public static Robot getInstance()
  {
    return instance;
  }

  /**
   * This function is run when the robot is first started up and should be used for any initialization code.
   */
  @Override
  public void robotInit()
  {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    // Create a timer to disable motor brake a few seconds after disable.  This will let the robot stop
    // immediately when disabled, but then also let it be pushed more 
    disabledTimer = new Timer();

    if (isSimulation())
    {
      DriverStation.silenceJoystickConnectionWarning(true);
    }

    SmartDashboard.putBoolean("Match:Shoot Now", false);
    SmartDashboard.putBoolean("Match:About to shoot", false);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics that you want ran
   * during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic()
  {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /**
   * Use the match timer to determine if we can shoot and notify 5 seconds before we can shoot again
   */
  private void updateAbleToShoot()
  {
    //If == U we do not know who won yet
    if (AUTO_WINNER_CODE == 'U')
    {
      String gameData;
      gameData = DriverStation.getGameSpecificMessage();
      if(gameData.length() > 0)
      {
        switch (gameData.charAt(0))
         {
          case 'B' : 
            //Blue case code
            AUTO_WINNER_CODE = 'B';
            break;
          case 'R' :
            //Red case code
            AUTO_WINNER_CODE = 'R';
            break;
          default :
            //This is corrupt data
            break;
        }
      } else {
        //Code for no data received yet
      }      
    }

    double timeLeftInTeleop = DriverStation.getMatchTime();
    if (timeLeftInTeleop >= 130) /*2:10 */
    {
      //In this timeperiod both sides can shoot
      RobotContainer.ABOUT_TO_BE_ABLE_TO_SHOOT = false;
      RobotContainer.CAN_SHOOT = true;
    }
    else if ((timeLeftInTeleop < 130 && timeLeftInTeleop >= 105) /*1:45 */ && ((AUTO_WINNER_CODE == 'B' && isRedAlliance) || (AUTO_WINNER_CODE == 'R' && !isRedAlliance)))
    {
      //The losers of auto shots right away
      RobotContainer.ABOUT_TO_BE_ABLE_TO_SHOOT = false;
      RobotContainer.CAN_SHOOT = true;
    }
    else if ((timeLeftInTeleop <= 110 /*1:50 */ && timeLeftInTeleop >= 105 /*1:05 */))
    {
      //winners of auto and there is five seconds left before we can shoot
      RobotContainer.ABOUT_TO_BE_ABLE_TO_SHOOT = true;
      RobotContainer.CAN_SHOOT = false;
    }
    else if ((timeLeftInTeleop < 105 && timeLeftInTeleop >= 80)/*1:20*/ && ((AUTO_WINNER_CODE == 'R' && isRedAlliance) || (AUTO_WINNER_CODE == 'B' && !isRedAlliance)))
    {
      //The winner of auto gets to shoot next
      RobotContainer.ABOUT_TO_BE_ABLE_TO_SHOOT = false;
      RobotContainer.CAN_SHOOT = true;
    }
    else if ((timeLeftInTeleop <= 85 /* 1:25 */ && timeLeftInTeleop >= 80 /* 1:20 */))
    {
      //Losers of auto now can be told about to be able shoot
      RobotContainer.ABOUT_TO_BE_ABLE_TO_SHOOT = true;
      RobotContainer.CAN_SHOOT = false;
    }
    else if ((timeLeftInTeleop < 80 && timeLeftInTeleop >= 55) && ((AUTO_WINNER_CODE == 'B' && isRedAlliance) || (AUTO_WINNER_CODE == 'R' && !isRedAlliance)))
    {
      //The loser of auto gets to shoot next
      RobotContainer.ABOUT_TO_BE_ABLE_TO_SHOOT = false;
      RobotContainer.CAN_SHOOT = true;
    }
    else if ((timeLeftInTeleop <= 60 /* 1:00 */ && timeLeftInTeleop >= 55 ))
    {
      //Winners of auto now can be told about to be able shoot
      RobotContainer.ABOUT_TO_BE_ABLE_TO_SHOOT = true;
      RobotContainer.CAN_SHOOT = false;
    }
    else if ((timeLeftInTeleop < 55 && timeLeftInTeleop >= 30) && ((AUTO_WINNER_CODE == 'R' && isRedAlliance) || (AUTO_WINNER_CODE == 'B' && !isRedAlliance)))
    {
      //The winners of auto gets to shoot next
      RobotContainer.ABOUT_TO_BE_ABLE_TO_SHOOT = false;
      RobotContainer.CAN_SHOOT = true;
    }
    else if ((timeLeftInTeleop <= 35 && timeLeftInTeleop >= 30 ))
    {
      //losers of auto now can be told about to be able shoot
      RobotContainer.ABOUT_TO_BE_ABLE_TO_SHOOT = true;
      RobotContainer.CAN_SHOOT = false;
    }
    else if ((timeLeftInTeleop < 30))
    {
      //everyone can shoot
      RobotContainer.ABOUT_TO_BE_ABLE_TO_SHOOT = false;
      RobotContainer.CAN_SHOOT = true;
    }
    else
    {
      //Default condition not able to shoot and no warning needed
      RobotContainer.CAN_SHOOT = false;      
      RobotContainer.ABOUT_TO_BE_ABLE_TO_SHOOT = false;      
    }
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit()
  {
    m_robotContainer.setMotorBrake(true);
    disabledTimer.reset();
    disabledTimer.start();
    //Disable auto aim
    RobotContainer.turretSubsystem.disableAutoAim();
  }
  
  @Override
  public void disabledPeriodic()
  {
    if (disabledTimer.hasElapsed(Constants.DrivebaseConstants.WHEEL_LOCK_TIME))
    {
      m_robotContainer.setMotorBrake(false);
      disabledTimer.stop();
      disabledTimer.reset();
    }
    //Disable auto aim
    RobotContainer.turretSubsystem.disableAutoAim();
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit()
  {
    updateAlliance();

    m_robotContainer.setMotorBrake(true);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    //Assign robot position
    if (isRedAlliance)
      RobotContainer.drivebase.resetOdometry(Constants.DrivebaseConstants.INITITAL_RED_POSE);
    else
      RobotContainer.drivebase.resetOdometry(Constants.DrivebaseConstants.INITITAL_BLUE_POSE);


    //Print the selected autonomous command upon autonomous init
    System.out.println("Auto selected: " + m_autonomousCommand);

    // schedule the autonomous command selected in the autoChooser
    if (m_autonomousCommand != null)
    {
      //Replace old command with new model
      // m_autonomousCommand.schedule();
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
    //Mark that we ran in auto
    hasRunAuto = true;
    //Enable auto aim
    RobotContainer.turretSubsystem.zeroEncoder();
    RobotContainer.turretSubsystem.enableAutoAim();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic()
  {
  }

  @Override
  public void teleopInit()
  {
    updateAlliance();
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.cancel();
    } else
    {
      CommandScheduler.getInstance().cancelAll();
    }
    //If we have not run auto set up initial pose
    if (!hasRunAuto)
    {
      if (isRedAlliance)
        RobotContainer.drivebase.resetOdometry(Constants.DrivebaseConstants.INITITAL_RED_POSE);
      else
        RobotContainer.drivebase.resetOdometry(Constants.DrivebaseConstants.INITITAL_BLUE_POSE);
      RobotContainer.turretSubsystem.zeroEncoder();
    }

    //Enable auto aim
    RobotContainer.turretSubsystem.enableAutoAim();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic()
  {
    //Get the winner of auto
    //TODO need to build a timer check to see when "we" can score
    updateAbleToShoot();    

    //Update Shuffleboard, TODO - Have LED routine light /flash leds
    SmartDashboard.putBoolean("Match:Shoot Now", RobotContainer.CAN_SHOOT);
    SmartDashboard.putBoolean("Match:About to shoot", RobotContainer.ABOUT_TO_BE_ABLE_TO_SHOOT);

    //adjust tweaked distance with joystick slider
    RobotContainer.launchSubsystem.updateTweakDistance(RobotContainer.driverJoystick.getRawAxis(3));
  }


  @Override
  public void testInit()
  {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic()
  {
  }

  /**
   * This function is called once when the robot is first started up.
   */
  @Override
  public void simulationInit()
  {
  }

  /**
   * This function is called periodically whilst in simulation.
   */
  @Override
  public void simulationPeriodic()
  {
  }

  public void updateAlliance()
  {
    Optional<Alliance> myAlliance = DriverStation.getAlliance();
    if(myAlliance.isPresent())
    {
      isRedAlliance = myAlliance.get() == Alliance.Red;
    }
  }

}