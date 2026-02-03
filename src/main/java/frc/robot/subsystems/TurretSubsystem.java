package frc.robot.subsystems;

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

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase {
     
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
    
    private double angleSetpoint = 0;
    private double x_currentAngleRot2Degree = 0;

    public TurretSubsystem() 
    {
        // angleSetpoint = absAngleEncoder.getPosition(); //Gets position in rotations        

        turretEncoder = m_motor.getEncoder();
        turretEncoder.setPosition(angleSetpoint); //update the position on motor encoder
        
        m_baseConfig.closedLoop
                        .p(kP)
                        .i(kI)
                        .d(kD)
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        .outputRange((-1 * maxOutput),maxOutput)
                        .maxMotion.maxAcceleration(Constants.TurretConstants.MAX_ACCELERATION)
                        ; // set PID
        m_baseConfig.idleMode(IdleMode.kBrake);
        
        m_baseConfig.encoder.positionConversionFactor(kTurretGearRatio);

        //Update the motoro config to use PID
        m_motor.configure(m_baseConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        // Initialize dashboard values
        SmartDashboard.setDefaultNumber("Turret Target Position", 0);
        SmartDashboard.setDefaultNumber("Turret Target Velocity", 0);
        SmartDashboard.setDefaultBoolean("Turret GO", false);
        SmartDashboard.setDefaultBoolean("Turret Alignment Trigger", false);
        SmartDashboard.putNumber("Turret P Gain", kP);
        SmartDashboard.putNumber("Turret I Gain", kI);
        SmartDashboard.putNumber("Turret D Gain", kD);
        SmartDashboard.putNumber("Turret IAccum", 0);
        SmartDashboard.putNumber("Turret MaxAccel", Constants.TurretConstants.MAX_ACCELERATION);
        SmartDashboard.setDefaultBoolean("Turret Stop", false);
    }

    
    public void setAngleMotor(double speed)
    {
        if (speed > 1) speed = 1;
        else if (speed < -1) speed = -1;
        m_motor.setVoltage(1 * speed);
    }

    /**
     * Stops the motor
     */
    public void stop()
    {
        //set the current
        double currentAngle = getTurretDegrees(turretEncoder.getPosition());
        closedLoopController.setSetpoint(currentAngle, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        closedLoopController.setIAccum(0);
        //turn off motor
        setAngleMotor(0);

        //turn off the control mode
        SmartDashboard.putBoolean("Turret GO", false);
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
    
    public double getCurrentTurretAngle()
    {
        return x_currentAngleRot2Degree;        
    }
    public void setCurrentTurretAngle(double angle)
    {
        x_currentAngleRot2Degree = angle;
    }
    public void zeroEncoder()
    {
        turretEncoder.setPosition(0);
    }

    public void periodic() 
    {    
        double currentMotorRotations = turretEncoder.getPosition();
        double currentAngleRot2Degree = getTurretDegrees(currentMotorRotations);
        //Update alignment trigger data
        boolean alignmentTrigger = turretAlignmentSwitch.get();
        SmartDashboard.putBoolean("Turret Alignment Trigger", alignmentTrigger);
        
        if (alignmentTrigger == true){
            currentAngleRot2Degree = 15;
            currentMotorRotations = getTurretRotations(currentAngleRot2Degree);
            turretEncoder.setPosition(currentMotorRotations);
            
        } 

        //Update negative values back to positoive
        if (currentAngleRot2Degree < 0) currentAngleRot2Degree = 360+currentAngleRot2Degree;
        setCurrentTurretAngle(currentAngleRot2Degree);
        SmartDashboard.putNumber("Turret Relative Angle rot2deg", currentAngleRot2Degree);
        SmartDashboard.putNumber("Turret IAccum", closedLoopController.getIAccum());

        if (SmartDashboard.getBoolean("Turret Stop", false)) 
        {
            SmartDashboard.putBoolean("Turret Stop", false);
            //Turn off motor
            stop();
        }
        else if (SmartDashboard.getBoolean("Turret GO", false)) 
        {
            SmartDashboard.putBoolean("Turret GO", false);
            /*
            * Get the target position from SmartDashboard and set it as the setpoint
            * for the closed loop controller.
            */
            double targetPositionDegrees = SmartDashboard.getNumber("Turret Target Position", angleSetpoint);
            targetPositionDegrees = targetPositionDegrees % 360;;
            if (targetPositionDegrees < 0) {
                //convert from negative rotation to positive rotation degrees
                targetPositionDegrees = 360 + targetPositionDegrees;
            }
            SmartDashboard.putNumber("Turret Target Position",targetPositionDegrees);
            
            //get the optimal target in rotations                
            double targetPositionRotations = getTargetRotationsFromDegrees(targetPositionDegrees);
            double maxAccel =  SmartDashboard.getNumber("Turret MaxAccel",0);

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

            //Run the motor
            closedLoopController.setSetpoint(targetPositionRotations, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        }
    }
    public void setTurretSetPoint(double setPointAngle)
    {
        angleSetpoint = setPointAngle;
          
        //SmartDashboard.putNumber("Turret Target Position",setPointAngle);
        //SmartDashboard.putBoolean("Turret GO", true);
    }

}