package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
    //private final VictorSP armMotor;
    //private final Encoder armEncoder;
    //private final ProfiledPIDController armController;

    // PID Gains and Motion Profile Constraints
    private static double kP = 0.5; // Proportional gain
    private static double kI = 0.0001; // Integral gain
    private static double kD = 0.0; // Derivative gain
    private static double maxOutput = 0.3;
    private static final double kMaxVelocity = 1.0; // Max velocity in units/sec
    private static final double kMaxAcceleration = 0.5; // Max acceleration in units/sec^2

    private SparkMax m_motor = new SparkMax(Constants.ArmConstants.ARM_MOTOR_ID, MotorType.kBrushless);
    public SparkAbsoluteEncoder absAngleEncoder = m_motor.getAbsoluteEncoder();
       
    //private SparkMaxConfig m_config = new SparkMaxConfig();
    private SparkMaxConfig m_baseConfig = new SparkMaxConfig();
    private SparkClosedLoopController closedLoopController = m_motor.getClosedLoopController();
     private RelativeEncoder encoder;
    
       
    private double angleSetpoint = 5;


    public ArmSubsystem() 
    {
        angleSetpoint = absAngleEncoder.getPosition(); //Gets position in rotations

        encoder = m_motor.getEncoder();
        encoder.setPosition(angleSetpoint); //update the position on motor encoder
        
        m_baseConfig.closedLoop
                        .p(kP)
                        .i(kI)
                        .d(kD)
                        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                        .outputRange((-1 * maxOutput),maxOutput); // set PID and 1/3 max speeds
        m_baseConfig.idleMode(IdleMode.kBrake);

        //Update the motoro config to use PID
        m_motor.configure(m_baseConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

            // Initialize dashboard values
        SmartDashboard.setDefaultNumber("Target Position", 0);
        SmartDashboard.setDefaultNumber("Target Velocity", 0);
        SmartDashboard.setDefaultBoolean("GO", false);
        SmartDashboard.setDefaultBoolean("Reset Encoder", false);
        SmartDashboard.setDefaultNumber("Absolute Angle", 0);
        SmartDashboard.putNumber("P Gain", kP);
        SmartDashboard.putNumber("I Gain", kI);
        SmartDashboard.putNumber("D Gain", kD);
        SmartDashboard.putNumber("IAccum", 0);
        SmartDashboard.setDefaultBoolean("Stop", false);

        Shuffleboard.getTab("Arm Sysid Testing").addDouble("Absolute Angle", absAngleEncoder::getPosition);
        Shuffleboard.getTab("Arm Sysid Testing").addDouble("Angle ProfileGoal", () -> angleSetpoint);
        Shuffleboard.getTab("Arm Sysid Testing").addDouble("Angle Motor Current", m_motor::getOutputCurrent);        
        Shuffleboard.getTab("Arm Sysid Testing").addDouble("Angle Motor Output", m_motor::getAppliedOutput);
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
        double currentAngle = Units.rotationsToDegrees(absAngleEncoder.getPosition());
        closedLoopController.setSetpoint(currentAngle, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        closedLoopController.setIAccum(0);
        //turn off motor
        setAngleMotor(0);

        //turn off the control mode
        SmartDashboard.putBoolean("GO", false);
    }
    

    public void periodic() 
    {    
        // Display encoder position and velocity
        SmartDashboard.putNumber("Actual Position", encoder.getPosition());
        SmartDashboard.putNumber("Actual Velocity", encoder.getVelocity());
        double currentAngleRot2Degree = Units.rotationsToDegrees(absAngleEncoder.getPosition());
        SmartDashboard.putNumber("Absolute Angle rot2deg", currentAngleRot2Degree);
        SmartDashboard.putNumber("IAccum", closedLoopController.getIAccum());

        if (SmartDashboard.getBoolean("Reset Encoder", false)) {
            SmartDashboard.putBoolean("Reset Encoder", false);
            // Reset the encoder position to abs value
            encoder.setPosition(absAngleEncoder.getPosition());
            SmartDashboard.putNumber("Target Position",currentAngleRot2Degree);
        }
        if (SmartDashboard.getBoolean("Stop", false)) 
        {
            SmartDashboard.putBoolean("Stop", false);
            //Turn off motor
            stop();
        }
        else if (SmartDashboard.getBoolean("GO", false)) 
        {
            SmartDashboard.putBoolean("GO", false);
            /*
            * Get the target position from SmartDashboard and set it as the setpoint
            * for the closed loop controller.
            */
            double targetPosition = SmartDashboard.getNumber("Target Position", 0);
            if (targetPosition > 360) {
                targetPosition = 360;
                SmartDashboard.putNumber("Target Position",targetPosition);
            } else if (targetPosition < 0) {
                targetPosition = 0;
                SmartDashboard.putNumber("Target Position",targetPosition);
            }
            double targetPositionRotations = Units.degreesToRotations(targetPosition);

            //Read PID values
            // read PID coefficients from SmartDashboard
            double p = SmartDashboard.getNumber("P Gain", 0);
            double i = SmartDashboard.getNumber("I Gain", 0);
            double d = SmartDashboard.getNumber("D Gain", 0);
            
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
                        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                        .outputRange((-1 * maxOutput),maxOutput); // set PID and 1/3 max speeds
                //Update the motoro config to use PID
                m_motor.configure(m_baseConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
            } //end if updatePID

            //Run the motor
            closedLoopController.setSetpoint(targetPositionRotations, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        }
    }

 
}