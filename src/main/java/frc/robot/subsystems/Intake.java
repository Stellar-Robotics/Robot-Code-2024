package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class Intake {

    private final CANSparkMax intake1;
    private final RelativeEncoder intake1_encoder;

    public Intake(int intakeCANId1) { // Constructor Function

        // Define Motors
        intake1 = new CANSparkMax(intakeCANId1, MotorType.kBrushless);

        // Reset motor controllers to factory settings
        intake1.restoreFactoryDefaults();

        // Grab the encoders of the motors
        intake1_encoder = intake1.getEncoder();

        // Motor encoder inversion option
        intake1_encoder.setInverted(false);

        // Flash motor configuration to the controllers
        intake1.burnFlash();

    }

    
    // Power Based Control
    public void setSpeed(double speed) {
        intake1.set(speed);
    }

    public double getMotorSpeed() {
        return intake1.get();
    }

    public void stop() {
        intake1.set(0);
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
