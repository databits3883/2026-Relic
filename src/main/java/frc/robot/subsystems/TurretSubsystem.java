package frc.robot.subsystems;

import org.photonvision.PhotonUtils;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class TurretSubsystem extends SubsystemBase {
     
    private static boolean CALIBRATION_MODE = true;

    // PID Gains and Motion Profile Constraints
    private static double kP = Constants.TurretConstants.KP;
    private static double kI = Constants.TurretConstants.KI;
    private static double kD = Constants.TurretConstants.KD;
    private static double maxOutput = Constants.TurretConstants.MAX_OUTPUT;
    private static final double kTurretGearRatio = Constants.TurretConstants.TURRET_GEAR_RATIO;
    
    private SparkMax m_motor = new SparkMax(Constants.TurretConstants.TURRET_MOTOR_ID, MotorType.kBrushless);

    //Set up the "zero" alignment triffer
    private DigitalInput turretAlignmentSwitch = new DigitalInput(0);    
       
    //private SparkMaxConfig m_config = new SparkMaxConfig();
    private SparkMaxConfig m_baseConfig = new SparkMaxConfig();
    private SparkClosedLoopController closedLoopController = m_motor.getClosedLoopController();
    private RelativeEncoder turretEncoder;
    
    //Default the turret to start at the angle defined in the constants
    private double angleSetpoint = Constants.TurretConstants.START_TURRET_ANGLE;
    
    private boolean isTurretEnabled = false;
    private boolean isAutoAiming = false;

    //Auto Aim variables
    private double targetAngle = 0;
    private Pose2d targetPose = null;
    private SwerveSubsystem swerveSubsystem = RobotContainer.drivebase;
    private boolean inPlayerArea = false;

    //Set up the target Poses of places turrent should aim to
    private Pose2d redHubPose = Constants.TurretConstants.RED_HUB_POSE; 
    private Pose2d redBottomPose = Constants.TurretConstants.RED_BOTTOM_POSE;
    private Pose2d redTopPose = Constants.TurretConstants.RED_TOP_POSE;
    private Pose2d blueBottomPose = Constants.TurretConstants.BLUE_BOTTOM_POSE;
    private Pose2d blueHubPose = Constants.TurretConstants.BLUE_HUB_POSE;
    private Pose2d blueTopPose = Constants.TurretConstants.BLUE_TOP_POSE;
  
    //These are the X, and Y values for the midline and the scoring zones
    private double midFieldY = Constants.TurretConstants.MID_FIELD_Y;
    private double redXPlayer = Constants.TurretConstants.RED_X_PLAYER;
    private double blueXPlayer = Constants.TurretConstants.BLUE_X_PLAYER;

    private long lastOuput = System.currentTimeMillis();
    private int lastTaget = 0;

    //Track the distance to the current target
    private double distanceToTarget = 0;

    //Set up the field object to allow us to show the current target on the field
    private final Field2d m_field = new Field2d();

    public TurretSubsystem() 
    {
        turretEncoder = m_motor.getEncoder();
        turretEncoder.setPosition(getTargetRotationsFromDegrees(angleSetpoint)); //update the position on motor encoder
        isTurretEnabled = false;
        
        m_baseConfig.closedLoop
                        .p(kP)
                        .i(kI)
                        .d(kD)
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        .outputRange((-1 * maxOutput),maxOutput)
                        .maxMotion.maxAcceleration(Constants.TurretConstants.MAX_ACCELERATION)
                        ; // set PID
        m_baseConfig.idleMode(IdleMode.kCoast);
        
        m_baseConfig.encoder.positionConversionFactor(kTurretGearRatio);

        //Update the motoro config to use PID
        m_motor.configure(m_baseConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        //set the setpoint to the current location
        closedLoopController.setSetpoint(turretEncoder.getPosition(), ControlType.kPosition, ClosedLoopSlot.kSlot0);

        //Aim system
        //Set up the field positions
        midFieldY = redHubPose.getY();
        redXPlayer = redHubPose.getX();
        blueXPlayer = blueHubPose.getX();

        //find target to aim based on the robot position
        targetPose = findTargetToAim(swerveSubsystem.getPose());

        // Initialize dashboard values
        SmartDashboard.putData("Field",m_field);
        SmartDashboard.setDefaultNumber("Turret Target Position", 0);
        SmartDashboard.setDefaultBoolean("Turret Alignment Trigger", false);
        SmartDashboard.putNumber("Turret P Gain", kP);
        SmartDashboard.putNumber("Turret I Gain", kI);
        SmartDashboard.putNumber("Turret D Gain", kD);
        SmartDashboard.putNumber("Turret IAccum", 0);
        SmartDashboard.putNumber("Turret MaxAccel", Constants.TurretConstants.MAX_ACCELERATION);
        SmartDashboard.setDefaultBoolean("Turret Stop", false);
        SmartDashboard.putNumber("Turret Target Position",angleSetpoint);
        SmartDashboard.putNumber("Turret Distance To Target",distanceToTarget);
        SmartDashboard.putNumber("Turret Motor Current",0);
    }
    
    /**
     * Used to set the turret to a voltage instead of a position
     * @param speed
     */
    public void setAngleMotor(double speed)
    {
        if (speed > 1) speed = 1;
        else if (speed < -1) speed = -1;
        m_motor.setVoltage(1 * speed);
        //Turn on the turret if a speed other than 0 is given
        if (speed != 0) isTurretEnabled = true;
    }

    /**
     * Stops the motor
     */
    public void stop()
    {
        //set the current setpoint to the current angle
        double currentAngle = getTurretDegrees(turretEncoder.getPosition());
        closedLoopController.setSetpoint(currentAngle, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        //Clear the i buildup
        closedLoopController.setIAccum(0);
        //turn off motor
        setAngleMotor(0);
        //Set target distance to zero
        distanceToTarget = 0;
        
        //turn off the control mode
        isTurretEnabled = false;
        //turn off auto aiming
        isAutoAiming = false;
    }

    /**
     * This will convert the motors rotations into a -359 - 359 degree for the turret
     * @param motorRotations
     * @return
     */
    private double getTurretDegrees(double motorRotations)
    {
        //Get the raw degrees given the rotations
        double turretDegrees = Units.rotationsToDegrees(motorRotations);
        //Reduce to -359 - 359
        turretDegrees = turretDegrees % 360;

        return (turretDegrees);
    }

    /** 
     * Get the turret's position in rotations
     */
    private double getTurretRotations(double turretDegrees)
    {
        //get normal rotations from degrees
        double rotations = Units.degreesToRotations(turretDegrees);

        return rotations;
    }

    /**
     * This will find the optimal delta angle to move the motor to get to the target.
     * It then adds/subtracts this in rotations to the current motors position
     * @param targetDegree
     * @return
     */
    private double getTargetRotationsFromDegrees(double targetDegree)
    {
        //Get the current rotations from the motorController
        double currentRotations = turretEncoder.getPosition();
        double currentAngleDegrees = getTurretDegrees(currentRotations);

        //If negative convert to 0-359
        if (currentAngleDegrees < 0) currentAngleDegrees = 360+currentAngleDegrees;

        //Now find delta of current Angle and target angle
        double deltaAngle = targetDegree - currentAngleDegrees;
        //If delta > 180, subtract 360 to get obtimal delta and reverse for negative
        if (deltaAngle < -180) deltaAngle = deltaAngle + 360;
        if (deltaAngle > 180) deltaAngle = deltaAngle - 360;

        //Get rotation of deltaAngle in Rotations
        double deltaRotations = getTurretRotations(deltaAngle);
        //Finally add this value to the current rotations for the new target
        double targetRotations = currentRotations + deltaRotations;

        return targetRotations;
    }
    
    /**
     * Return the current turret angle from the encoder
     * @return
     */
    public double getCurrentTurretAngle()
    {
        return getTurretDegrees(turretEncoder.getPosition());        
    }
    
    /**
     * Sets the turret to zero, probably should change this to match the alignment constant
     */
    public void zeroEncoder()
    {
        turretEncoder.setPosition(0);
    }

    /**
     * Finds the best target and sets the turret to aim there
     */
    private void autoAimToBestTarget()
    {
        Pose2d robotPose = swerveSubsystem.getPose();

        //Transform robot pose
        Pose2d turretPose = robotPose.plus(Constants.TurretConstants.CENTER_TURRET_FROM_CENTER_BOT_RIGHT);

        //Find new target based on robot positon
        targetPose = findTargetToAim(turretPose);    
        targetAngle = getAngle(turretPose, targetPose);
        //Use PhotonVision helper method to get distance
        distanceToTarget = PhotonUtils.getDistanceToPose(turretPose, targetPose);
        //update the turret setpoint
        setTurretSetPoint(targetAngle);
    }

    /**
     * Returns the distance to the current tracked target, 0 means no target to autoaim is off
     * @return
     */
    public double getDistanceToTarget()
    {
        return distanceToTarget;
    }
    
    /**
     * This runs every 10ms or so
     */
    public void periodic() 
    {    
        double currentMotorRotations = turretEncoder.getPosition();
        double currentAngleRot2Degree = getTurretDegrees(currentMotorRotations);
        //Update alignment trigger data
        boolean alignmentTrigger = turretAlignmentSwitch.get();
        
        //if the alignment switch is triggered force the turret encoder to update its position
        //no wires on the DIO is true, so switch will set FALSE
        if (alignmentTrigger == false)
        {
            currentAngleRot2Degree = Constants.TurretConstants.ALIGNMENT_SWITCH_ANGLE;
            currentMotorRotations = getTurretRotations(currentAngleRot2Degree);  
            if (CALIBRATION_MODE) System.out.println("Switch: previous position: "+ turretEncoder.getPosition() + " target.pos: " + closedLoopController.getSetpoint());                      
            turretEncoder.setPosition(currentMotorRotations);     
            //Since the encoder has been reset, define new set point based on new encoder position
            closedLoopController.setSetpoint(getTargetRotationsFromDegrees(angleSetpoint), ControlType.kPosition, ClosedLoopSlot.kSlot0);            
            if (CALIBRATION_MODE) System.out.println("Switch: current position: "+ turretEncoder.getPosition() + " target.pos: " + closedLoopController.getSetpoint());                      
        } 

        //Update negative values back to positoive
        if (currentAngleRot2Degree < 0) currentAngleRot2Degree = 360+currentAngleRot2Degree;
        //Update the angleSetPoint
        angleSetpoint = currentAngleRot2Degree;

        if (CALIBRATION_MODE)
        {
          SmartDashboard.putBoolean("Turret Alignment Trigger", alignmentTrigger);
          SmartDashboard.putNumber("Turret Relative Angle rot2deg", currentAngleRot2Degree);
          SmartDashboard.putNumber("Turret Encoder rots", turretEncoder.getPosition());
          SmartDashboard.putNumber("Turret SetPoint rots", closedLoopController.getSetpoint());
          SmartDashboard.putNumber("Turret Distance To Target",getDistanceToTarget());
          SmartDashboard.putNumber("Turret Motor Current",m_motor.getAppliedOutput());

        }

        if (SmartDashboard.getBoolean("Turret Stop", false)) 
        {
            SmartDashboard.putBoolean("Turret Stop", false);
            //Turn off motor
            stop();
        }
        else if (isTurretEnabled)
        {
            //check if auto aim is defined, if so get the new target
            if (isAutoAiming)
            {
                autoAimToBestTarget();
            }
            /*
            * Get the target position from SmartDashboard and set it as the setpoint
            * for the closed loop controller.
            */
            double targetPositionDegrees = this.angleSetpoint;
            //If we are in calibration mode read target from dashboard
            if (CALIBRATION_MODE) targetPositionDegrees = SmartDashboard.getNumber("Turret Target Position", angleSetpoint);
            
            targetPositionDegrees = targetPositionDegrees % 360;;
            if (targetPositionDegrees < 0) 
            {
                //convert from negative rotation to positive rotation degrees
                targetPositionDegrees = 360 + targetPositionDegrees;
            }
            if (CALIBRATION_MODE) SmartDashboard.putNumber("Turret Target Position",targetPositionDegrees);
            //Put target back into dashboard
            SmartDashboard.putNumber("Turret Target Position",targetPositionDegrees);
            
            //get the optimal target in rotations                
            double targetPositionRotations = getTargetRotationsFromDegrees(targetPositionDegrees);
            double maxAccel =  SmartDashboard.getNumber("Turret MaxAccel",0);

            if (CALIBRATION_MODE)
            {
                //Read PID values
                // read PID coefficients from SmartDashboard
                double p = SmartDashboard.getNumber("Turret P Gain", 0);
                double i = SmartDashboard.getNumber("Turret I Gain", 0);
                double d = SmartDashboard.getNumber("Turret D Gain", 0);
                
                // if PID coefficients on SmartDashboard have changed, write new values to controller
                boolean updatePID = false;
                if((p != kP)) { updatePID = true;  kP = p; }
                if((i != kI)) { updatePID = true;  kI = i; }
                if((d != kD)) { updatePID = true; kD = d; }

                if (updatePID)
                {
                    //Update the PID on close loopController
                    m_baseConfig.closedLoop
                            .p(kP)
                            .i(kI)
                            .d(kD)
                            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                            .outputRange((-1 * maxOutput),maxOutput)
                            .maxMotion.maxAcceleration(maxAccel)
                            ; // set PID
                    //Update the motoro config to use PID
                    m_motor.configure(m_baseConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
                } //end if updatePID
            } // end calibration mode check

            //Run the motor
            //Update the angleSetPoint
            angleSetpoint = getTurretDegrees(targetPositionRotations);
            closedLoopController.setSetpoint(targetPositionRotations, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        } // end if enabled
    }

    /**
     * Sets the turrent's set point
     * @param setPointAngle
     */
    public void setTurretSetPoint(double setPointAngle)
    {
        // If calibration mode is enabled set the target set point on the dashboard as well
        if (CALIBRATION_MODE) SmartDashboard.putNumber("Turret Target Position",setPointAngle);
        angleSetpoint = setPointAngle;
        isTurretEnabled = true;
    }
  /*
   * This will check the alliance and position of the robot
   * to determine what april tag the turret should point to
   */
  private Pose2d findTargetToAim(Pose2d turretPose)
  {
    Pose2d targetPose = null;
    boolean isRedAlliance = Robot.isRedAlliance;

    double turretX = turretPose.getX();
    double turretY = turretPose.getY();
     m_field.getObject("TurretPose").setPose(turretPose);

    if (isRedAlliance)
    {
      inPlayerArea = false;
      if (turretX > redXPlayer)
      {
        inPlayerArea = true;
        targetPose = redHubPose;
        if (this.lastTaget != 1)
        {
          System.out.println("picked target redHub");
          lastTaget = 1;
        }
      }
      else if (turretY > midFieldY)
      {
        targetPose = redTopPose;
        if (this.lastTaget != 2)
        {
            System.out.println("picked target redTop");
            lastTaget = 2;
        }
      }
      else
      {
        targetPose = redBottomPose;
        if (this.lastTaget != 3)
        {
            System.out.println("picked target redBottom");
            lastTaget = 3;
        }
      }
    } //end red alliance
    else
    {
      if (turretX < blueXPlayer)
      {
        inPlayerArea = true;
        targetPose = blueHubPose;
        if (this.lastTaget != 4)
        {
            System.out.println("picked target blueHub");
            lastTaget = 4;
        }
      }
      else if (turretY > midFieldY)
      {
        targetPose = blueTopPose;
        if (this.lastTaget != 5)
        {
            System.out.println("picked target blueTop");
            lastTaget = 5;
        }
      }
      else
      {
        targetPose = blueBottomPose;
        if (this.lastTaget != 6)
        {
            System.out.println("picked target bluBottom");
            lastTaget = 6;
        }
      }
    }  //End blue alliance
    //If no target is found turn off aim, do we need this??
    if (targetPose == null) isAutoAiming = false;
    m_field.getObject("Target").setPose(targetPose);
    return targetPose;
  }

  /**
   * Finds the angle between two points
   * Will need to update to also find distance between them
   * @param robotPose
   * @param targetPose
   * @return
   */
  private double getAngle(Pose2d robotPose, Pose2d targetPose)
  {
    //Get current robotRotation
    double robotHeadingDeg = swerveSubsystem.getHeading().getDegrees();
    //Need to know what to do with these angles, add heading?  Subtract heading?
    double targetX = targetPose.getX();
    double targetY = targetPose.getY();
    double robotX = robotPose.getX();
    double robotY = robotPose.getY();

    double theta = Math.atan2(targetY-robotY, targetX-robotX);
    //We need to invert this angle from clockwise to counterclockwise if we are not in player area
    //if (!inPlayerArea) theta = theta * -1;

    //Get the rotation from the center of the robot to the target if the robot was facing 0 (blue)
    double relativeRotationDeg = Units.radiansToDegrees(theta);
    StringBuffer output = new StringBuffer();
    output.append("[tx:").append(targetX).append(",")
          .append("ty:").append(targetY).append(",")
          .append("rx:").append(robotX).append(",")
          .append("ry:").append(robotY).append(",")
          .append("\ntargetDeg:").append(relativeRotationDeg).append(",");

    //Take robots position into account
    relativeRotationDeg -= robotHeadingDeg;
    output.append("targetDegRobot:").append(relativeRotationDeg).append(",");
    //Put back to -359 to 359 range
    relativeRotationDeg = relativeRotationDeg % 360;
    output.append("finalTargetDeg:").append(relativeRotationDeg).append(",");

    //only output debugging every second
    long currentTime = System.currentTimeMillis();
    long delta = currentTime - lastOuput;
    if (CALIBRATION_MODE && (delta > 1000))
    {
        System.out.println(output.toString());
        lastOuput = currentTime;
    }

    //Right now set target to just the angle to tag
    return relativeRotationDeg;
  }

  public void disableAutoAim()
  {
    isAutoAiming = false;
    distanceToTarget = 0;
    //Should we stop the turret altogether, for now yes
    stop();
  }
  public void enableAutoAim()
  {
    isAutoAiming = true;
    isTurretEnabled = true;
  }
  public boolean isAutoAiming()
  {
    return isAutoAiming;
  }
  public void toggleAutoAim()
  {
    if (!isAutoAiming) enableAutoAim();
    else disableAutoAim();
  }

    
}