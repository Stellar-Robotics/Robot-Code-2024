package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import frc.robot.Constants.IntakeConstants;

public class Intake {

    // Declare Motors, encoders, and Position Controllers
    private final CANSparkMax intakeDrive;
    private final CANSparkMax intakeAngle;
    private final RelativeEncoder intakeAngleEncoder;

    // Using SparkPIDController instead of a generic one in order
    // to execute PID operatons on the angle motor controller.
    private final SparkPIDController angleController;

    public Intake() { // Constructor Function

        // Define Spark Motors
        intakeDrive = new CANSparkMax(IntakeConstants.intakeDriveControllerId, MotorType.kBrushless);
        intakeAngle = new CANSparkMax(IntakeConstants.intakeAngleControllerId, MotorType.kBrushless);

        // Reset motor controllers to factory settings
        intakeDrive.restoreFactoryDefaults();
        intakeAngle.restoreFactoryDefaults();

        // Set current limits
        intakeDrive.setSmartCurrentLimit(30, 30);
        intakeAngle.setSmartCurrentLimit(30, 30);

        // Grab the encoders of the motors
        intakeAngleEncoder = intakeAngle.getEncoder();

        this.resetAngleEncoder();

        // Flash motor configuration to the controllers
        intakeDrive.burnFlash();
        intakeAngle.burnFlash();

        // Define Position Controller for the Angle Motor
        angleController = intakeAngle.getPIDController();

        // Setting intial PID Values for the Angle
        angleController.setP(IntakeConstants.intakeAngleP);
        angleController.setI(IntakeConstants.intakeAngleI);
        angleController.setD(IntakeConstants.intakeAngleD);

    }

    
    // Intake drive motor setters.
    public void setDriveSpeed(double speed) {
        intakeDrive.set(speed);
    }

    public void stopDrive() {
        intakeDrive.set(0);
    }

    // Intake drive motor getter
    public double getDriveSpeed() {
        return intakeDrive.get();
    }

    // Angle Encoder operations
    public void resetAngleEncoder() {
        intakeAngleEncoder.setPosition(0);
    }

    public void setAngleEncoder(double position) {
        intakeAngleEncoder.setPosition(position);
    }

    public double getAngleEncoderPos() {
        return intakeAngleEncoder.getPosition();
    }

    // Angle Controller Setters
    public void setTargetAngle(double angleDegrees) {
        angleController.setReference(angleDegrees / 360, CANSparkMax.ControlType.kPosition);
    }
     
}
