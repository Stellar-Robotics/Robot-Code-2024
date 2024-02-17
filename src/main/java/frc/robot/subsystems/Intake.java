package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ModuleConstants;

import com.revrobotics.CANSparkBase;

public class Intake {

    // Declare Motors, encoders, and Position Controllers
    private final CANSparkMax intakeDrive;
    private final CANSparkMax intakeAngle;
    private final AbsoluteEncoder intakeAngleEncoder;

    // Using SparkPIDController instead of a generic one in order
    // to execute PID operatons on the angle motor controller.
    private final SparkPIDController angleController;

    boolean isExtended;

    public Intake() { // Constructor Function

        isExtended = false;

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

        angleController.setPositionPIDWrappingEnabled(true);
        angleController.setPositionPIDWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput);
        angleController.setPositionPIDWrappingMaxInput(1);

        // Flash motor configuration to the controllers
        intakeDrive.burnFlash();
        intakeAngle.burnFlash();

        angleController.setReference(0.05, CANSparkMax.ControlType.kPosition);
        

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

    // Angle Encoder operation
    public double getAngleEncoderPos() {
        return intakeAngleEncoder.getPosition();
    }

    // Angle Controller Setters
    public void setTargetAngle(double angleDegrees) {
        angleController.setReference(Math.min(Math.max(angleDegrees, IntakeConstants.intakeMinAngle), IntakeConstants.intakeMaxAngle), CANSparkMax.ControlType.kPosition);
    }

    // Switch the state up and down
    public void toggleState() {
        if (isExtended) {
            this.setTargetAngle(0.35);
        } else {
            this.setTargetAngle(0.05);
        }
        // Update our extension status
        isExtended = !isExtended;
    }
     
}
