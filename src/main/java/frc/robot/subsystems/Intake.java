package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.ShooterConstants;
import frc.utils.MiscUtils;

public class Intake {

    // Declare Motors, encoders, and Position Controllers
    private final CANSparkMax intakeDrive;
    private final CANSparkMax intakeAngle;
    private final AbsoluteEncoder intakeAngleEncoder;
    private double lastAngle;

    // Using SparkPIDController instead of a generic one in order
    // to execute PID operatons on the angle motor controller.
    private final SparkPIDController angleController;

    // A status variable for intake toggle functionality
    public boolean isExtended;

    public Intake() { // Constructor Function

        // Intake toggle starts in the false case
        isExtended = false;
        lastAngle = 0.6;

        // Define Spark Motors
        intakeDrive = new CANSparkMax(IntakeConstants.intakeDriveControllerId, MotorType.kBrushless);
        intakeAngle = new CANSparkMax(IntakeConstants.intakeAngleControllerId, MotorType.kBrushless);

        // Reset motor controllers to factory settings
        intakeDrive.restoreFactoryDefaults();

        // Were disabling the factory reset so we dont lose out absolute encoder software offeset.
        //intakeAngle.restoreFactoryDefaults();

        // Set current limits
        intakeDrive.setSmartCurrentLimit(30, 30);
        intakeAngle.setSmartCurrentLimit(30, 30);

        // Grab the encoders of the motors
        intakeAngleEncoder = intakeAngle.getAbsoluteEncoder(Type.kDutyCycle);

        // Define Position Controller for the Angle Motor
        angleController = intakeAngle.getPIDController();
        angleController.setFeedbackDevice(intakeAngleEncoder);

        // Setting intial PID Values for the Angle
        angleController.setP(IntakeConstants.intakeAngleP);
        angleController.setI(IntakeConstants.intakeAngleI);
        angleController.setD(IntakeConstants.intakeAngleD);
        angleController.setFF(0);
        angleController.setOutputRange(-1, 1);

        // Making sure our intake doesent go through our robot
        angleController.setPositionPIDWrappingEnabled(true);
        angleController.setPositionPIDWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput);
        angleController.setPositionPIDWrappingMaxInput(1);

        // Flash motor configuration to the controllers
        intakeDrive.burnFlash();
        intakeAngle.burnFlash();

        // Setting a starting position for the intake
        angleController.setReference(0.05, CANSparkMax.ControlType.kPosition);
        

    }

    
    // Intake drive motor setters.
    public void setDriveSpeed(double speed) {
        intakeDrive.set(speed);
    }

    // This only stops the motor when its in drive mode
    public void stopDrive() {
        intakeDrive.set(0);
    }

    // Intake drive motor getter (power mode only)
    public double getDriveSpeed() {
        return intakeDrive.get();
    }

    // Angle Encoder operation
    public double getAngleEncoderPos() {
        return intakeAngleEncoder.getPosition();
    }

    // Angle Controller Setters
    public void setTargetAngle(double angleDegrees) {
        angleController.setReference(MiscUtils.clamp(IntakeConstants.intakeMinAngle, IntakeConstants.intakeMaxAngle, angleDegrees), ControlType.kPosition);
    }

    // Incrament intake angle
    public void incramentAngle(double rotations) {
        lastAngle = MiscUtils.clamp(IntakeConstants.intakeMinAngle, IntakeConstants.intakeMaxAngle, lastAngle + rotations);
        this.setTargetAngle(lastAngle);
    }



    // Switch the state up and down
    public void toggleState() {
        if (isExtended) {
            this.setTargetAngle(0.35);
        } else {
            this.setTargetAngle(0.18);
        }
        // Update our extension status
        isExtended = !isExtended;
    }
     
}
