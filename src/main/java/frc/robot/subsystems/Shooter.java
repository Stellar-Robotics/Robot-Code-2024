package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
//import com.revrobotics.RelativeEncoder;

public class Shooter {

    private final CANSparkMax shooterMotorController;
    //private final RelativeEncoder intake1_encoder;

    public Shooter(int shooterCanId) { // Constructor Function

        // Define Motors
        shooterMotorController = new CANSparkMax(shooterCanId, MotorType.kBrushless);

        // Reset motor controllers to factory settings
        shooterMotorController.restoreFactoryDefaults();
        shooterMotorController.setSmartCurrentLimit(30, 30);


        // Grab the encoders of the motors
        //intake1_encoder = intake1.getEncoder();

        // Motor encoder inversion option
        //intake1_encoder.setInverted(false);

        // Flash motor configuration to the controllers
        shooterMotorController.burnFlash();

    }

    
    // Power Based Control
    public void setSpeed(double speed) {
        shooterMotorController.set(speed);
    }

    public double getMotorSpeed() {
        return shooterMotorController.get();
    }

    public void stop() {
        shooterMotorController.set(0);
    }

    // Encoder operations
    public void resetEncoder() {
        //intake1_encoder.setPosition(0);
    }

    public void setEncoder(double position) {
       // intake1_encoder.setPosition(position);
    }

    //public double getEncoderPos() {
       // return intake1_encoder.getPosition();
    //}
     
}
