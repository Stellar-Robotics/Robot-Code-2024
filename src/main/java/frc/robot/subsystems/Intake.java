package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class Intake {

    private final CANSparkMax intakeMotorController;
    private final RelativeEncoder intake1_encoder;

    public Intake(int intakeCanId) { // Constructor Function

        // Define Motors
        intakeMotorController = new CANSparkMax(intakeCanId, MotorType.kBrushless);

        // Reset motor controllers to factory settings
        intakeMotorController.restoreFactoryDefaults();

        // Grab the encoders of the motors
        intake1_encoder = intakeMotorController.getEncoder();


        // Flash motor configuration to the controllers
        intakeMotorController.burnFlash();

    }

    
    // Power Based Control
    public void setSpeed(double speed) {
        intakeMotorController.set(speed);
    }

    public double getMotorSpeed() {
        return intakeMotorController.get();
    }

    public void stop() {
        intakeMotorController.set(0);
    }

    // Encoder operations
    public void resetEncoder() {
        intake1_encoder.setPosition(0);
    }

    public void setEncoder(double position) {
        intake1_encoder.setPosition(position);
    }

    public double getEncoderPos() {
        return intake1_encoder.getPosition();
    }
     
}
