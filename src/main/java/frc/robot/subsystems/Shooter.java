package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import frc.robot.Constants.ShooterConstants;

public class Shooter {

    private final CANSparkMax shooterDriveController1;
    private final CANSparkMax shooterDriveController2;
    private final CANSparkMax shooterAngleController;

    private final RelativeEncoder shooterAngleEncoder;

    private final SparkPIDController shooterAnglePIDController;

    public Shooter() { // Constructor Function

        // Define Motors
        shooterDriveController1 = new CANSparkMax(ShooterConstants.shooterDriveControllerID1, MotorType.kBrushless);
        shooterDriveController2 = new CANSparkMax(ShooterConstants.shooterDriveControllerID2, MotorType.kBrushless);
        shooterAngleController = new CANSparkMax(ShooterConstants.shooterAngleControllerID, MotorType.kBrushless);

        // Reset motor controllers to factory settings
        shooterDriveController1.restoreFactoryDefaults();
        shooterDriveController2.restoreFactoryDefaults();
        shooterAngleController.restoreFactoryDefaults();

        // Set current limits
        shooterDriveController1.setSmartCurrentLimit(30, 30);
        shooterDriveController2.setSmartCurrentLimit(30, 30);
        shooterAngleController.setSmartCurrentLimit(30, 30);

        // Get the angle controller encoder
        shooterAngleEncoder = shooterAngleController.getEncoder();

        // Get the angle PID controller
        shooterAnglePIDController = shooterAngleController.getPIDController();

        // Set Angle motor PID values
        shooterAnglePIDController.setP(ShooterConstants.shooterAngleP);
        shooterAnglePIDController.setI(ShooterConstants.shooterAngleI);
        shooterAnglePIDController.setD(ShooterConstants.shooterAngleD);

        // Set PID Feedback Device
        shooterAnglePIDController.setFeedbackDevice(shooterAngleEncoder);

        // Flash motor configuration to the controllers
        shooterDriveController1.burnFlash();
        shooterDriveController2.burnFlash();
        shooterAngleController.burnFlash();

    }

    
    // Drive motor getters and setters
    public void setDriveSpeed(double speedPower) {
        shooterDriveController1.set(speedPower);
        shooterDriveController2.set(-speedPower);
    }

    public void stopDriveMotors() {
        shooterDriveController1.set(0);
        shooterDriveController2.set(0);
    }

    public double[] getDrivePower() {
        return new double[] {shooterDriveController1.get(), shooterDriveController2.get()};
    }

    // Shooter angle operations
    public void resetAngleEncoder() {
        shooterAngleEncoder.setPosition(0);
    }

    public void setAngleEncoder(double position) {
        shooterAngleEncoder.setPosition(position);
    }

    public double getAngleEncoderPos() {
        return shooterAngleEncoder.getPosition();
    }

    // Angle Controller Setters
    public void setTargetAngle(double angleRotations) {
        shooterAnglePIDController.setReference(Math.min(Math.max(ShooterConstants.shooterMinAngle, angleRotations), ShooterConstants.shooterMaxAngle), CANSparkMax.ControlType.kPosition);
    }
     
}
