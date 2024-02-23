package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ShooterConstants;
import frc.utils.MiscUtils;

public class Climber {
    private final CANSparkMax climberLeft;
    private final CANSparkMax climberRight;

    private final RelativeEncoder climberLeftEncoder;
    private final RelativeEncoder climberRightEncoder;

    private final SparkPIDController climberLeftPIDController;
    private final SparkPIDController climberRightPIDController;

    private double targetPosition = 0;



    public Climber() {

        // Decalare Motors and get encoders
        climberLeft = new CANSparkMax(ClimberConstants.climberLeftCANID, MotorType.kBrushless);
        climberRight = new CANSparkMax(ClimberConstants.climberRightCANID, MotorType.kBrushless);

        climberLeftEncoder = climberLeft.getEncoder();
        climberRightEncoder = climberRight.getEncoder();

        // Apply defaults before setting custom config values
        climberLeft.restoreFactoryDefaults();
        climberRight.restoreFactoryDefaults();

        climberLeft.setIdleMode(IdleMode.kBrake);
        climberRight.setIdleMode(IdleMode.kBrake);

        // Set current limits (IMPORTANT)
        climberLeft.setSmartCurrentLimit(30, 30);
        climberRight.setSmartCurrentLimit(30, 30);

        // PID Schuff
        climberLeftPIDController = climberLeft.getPIDController();
        climberRightPIDController = climberRight.getPIDController();

        climberLeftPIDController.setP(ShooterConstants.shooterAngleP);
        climberLeftPIDController.setI(ShooterConstants.shooterAngleI);
        climberLeftPIDController.setD(ShooterConstants.shooterAngleD);

        climberRightPIDController.setP(ShooterConstants.shooterAngleP);
        climberRightPIDController.setI(ShooterConstants.shooterAngleI);
        climberRightPIDController.setD(ShooterConstants.shooterAngleD);

        // Set PID Feedback Device
        climberLeftPIDController.setFeedbackDevice(climberLeftEncoder);
        climberRightPIDController.setFeedbackDevice(climberRightEncoder);

        climberLeft.setInverted(false);

        // Commit configs to controller EEPROM
        climberLeft.burnFlash();
        climberRight.burnFlash();

    }

    // Setters for power based control
    public void setRawPower(double powerLeft, double powerRight) {
        climberLeft.set(powerLeft);
        climberRight.set(powerRight);
    }

    public void setClimberLeftPower(double power) {
        climberLeft.set(power);
    }

    public void setClimberRightPower(double power) {
        climberRight.set(power);
    }

    // Position Setters for use in position mode
    public void setClimberLeftPosition(double powerRotations) {
        climberLeftPIDController.setReference(MiscUtils.clamp(0, ClimberConstants.halfEncoderTicks, powerRotations), CANSparkMax.ControlType.kPosition);
    }

    public void setClimberRightPosition(double powerRotations) {
        climberRightPIDController.setReference(MiscUtils.clamp(0, ClimberConstants.halfEncoderTicks, powerRotations), CANSparkMax.ControlType.kPosition);
    }

    public void incramentPosition(double speed) {
        // Prevent the incrament for going beyond the max range with a clamp
        targetPosition = MiscUtils.clamp(0, ClimberConstants.halfEncoderTicks, targetPosition + speed);
        this.setClimberLeftPosition(targetPosition);
        this.setClimberRightPosition(targetPosition);
    }

    // Getters
    public double[] getEncoderValues() {
        return new double[] {
            climberLeftEncoder.getPosition(),
            climberRightEncoder.getPosition()
        };
    }

}
