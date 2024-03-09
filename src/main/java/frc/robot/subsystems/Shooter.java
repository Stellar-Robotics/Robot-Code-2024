package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import frc.robot.subsystems.VisionSubsystem;

import frc.robot.Constants.ShooterConstants;
import frc.utils.MiscUtils;

public class Shooter {

    private final CANSparkMax shooterDriveController;
    private final CANSparkMax shooterAngleController;

    private final RelativeEncoder shooterAngleEncoder;
    private final RelativeEncoder shooterDriveEncoder;

    private final SparkPIDController shooterAnglePIDController;
    private final SparkPIDController shooterDrivePIDController;

    private VisionSubsystem vision;

    private double lastDriveSpeed = 0;
    private double lastAngle = 0;

    private final double SPEAKER_HEIGHT = 2.047;
    private final double APRIL_TAG_TO_FLOOR = 1.45;
    private final double Z_OFFSET = 0.2285; // meters

    private final double DEGREES_TO_ROTATIONS = 3; // TODO: THIS IS INCORRECT!! MEASURE THIS!!!

    public Shooter(VisionSubsystem vision) {

        // Define Motors
        shooterDriveController = new CANSparkMax(ShooterConstants.shooterDriveControllerID, MotorType.kBrushless);
        shooterAngleController = new CANSparkMax(ShooterConstants.shooterAngleControllerID, MotorType.kBrushless);

        // Reset motor controllers to factory settings
        shooterDriveController.restoreFactoryDefaults();
        shooterAngleController.restoreFactoryDefaults();

        // Set current
        shooterDriveController.setSmartCurrentLimit(40);
        shooterAngleController.setSmartCurrentLimit(30);

        // Get the angle controller encoder
        shooterAngleEncoder = shooterAngleController.getEncoder();
        shooterDriveEncoder = shooterDriveController.getEncoder();

        // Get the angle PID controller
        shooterAnglePIDController = shooterAngleController.getPIDController();
        shooterDrivePIDController = shooterDriveController.getPIDController();

        // Set Angle motor PID values
        shooterAnglePIDController.setP(ShooterConstants.shooterAngleP);
        shooterAnglePIDController.setI(ShooterConstants.shooterAngleI);
        shooterAnglePIDController.setD(ShooterConstants.shooterAngleD);

        shooterDrivePIDController.setP(ShooterConstants.shooterDriveP);
        shooterDrivePIDController.setI(ShooterConstants.shooterDriveI);
        shooterDrivePIDController.setD(ShooterConstants.shooterDriveD);

        // Set PID Feedback Device
        shooterAnglePIDController.setFeedbackDevice(shooterAngleEncoder);
        shooterDrivePIDController.setFeedbackDevice(shooterDriveEncoder);
        shooterAngleController.setInverted(true);
        shooterDriveController.setInverted(true);

        // Flash motor configuration to the controllers
        shooterDriveController.burnFlash();
        shooterAngleController.burnFlash();

        this.vision = vision;
    }

    
    // Drive motor getters and setters
    public void setDrivePower(double speedPower) {
        shooterDriveController.set(speedPower);
    }

    public void stopDriveMotors() {
        //shooterDriveController.set(0);
    }

    public double getDrivePower() {
        return shooterDriveController.get();
    }

    // Shooter encoder operations
    public void resetAngleEncoder() {
        shooterAngleEncoder.setPosition(0);
    }

    public void setAngleEncoder(double position) {
        shooterAngleEncoder.setPosition(position);
    }

    public double getAngleEncoderPos() {
        return shooterAngleEncoder.getPosition();
    }

    // Driver velocity control setters
    public void setDriveSpeed(double speedRPMs) { // Dividing to adjust for gearing
        shooterDrivePIDController.setReference(speedRPMs, ControlType.kVelocity);
    }

    public void incramentDriveSpeed(double speedRPMs) {
        lastDriveSpeed = MiscUtils.clamp(-300, 300, lastDriveSpeed + (speedRPMs/50));
        this.setDriveSpeed(lastDriveSpeed);

        this.shooterDriveController.set(1);
    }

    public void resetDriveSpeed() {
        lastDriveSpeed = 0;
        this.setDriveSpeed(0);
    }

    // Angle position control setters
    public void setTargetAngle(double angleRotations) {

        if (Double.isNaN(angleRotations)) {
            return;
        }
        shooterAnglePIDController.setReference(MiscUtils.clamp(ShooterConstants.shooterMinAngle, ShooterConstants.shooterMaxAngle, angleRotations), CANSparkMax.ControlType.kPosition);
    }

    public void setTargetAngleDegrees(double degrees) {
        this.setTargetAngle((degrees - 34) * DEGREES_TO_ROTATIONS);
    }

    public void setVisionAngle() {
        double dSquared = Math.pow(vision.getAprilTagZ(0), 2); // Where d = distance from april tag to camera (refrenced as apriltag z)
        double nSquared = Math.pow(this.APRIL_TAG_TO_FLOOR, 2); // self explanitory (if it wasn't already obvious)
        double z = Math.sqrt(dSquared - nSquared); // distance from the robot to the front face of the speaker
        this.setAngleFromDistance(z);
    }

    public void setAngleFromDistance(double distance) {
        this.setTargetAngleDegrees(Math.toDegrees(Math.atan(SPEAKER_HEIGHT / (distance - Z_OFFSET))));
    }

    public void incramentAngle(double rotations) {
        lastAngle = MiscUtils.clamp(ShooterConstants.shooterMinAngle, ShooterConstants.shooterMaxAngle, lastAngle + rotations);
        this.setTargetAngle(lastAngle);
    }

    public void executePreset(double position, double speed) {
        this.setTargetAngle(position);
        this.setDriveSpeed(speed);
    }
     
}
