package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import java.lang.reflect.Array;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import frc.robot.Constants.ClimberConstants;

public class Climber {
    private final CANSparkMax climberLeft;
    private final CANSparkMax climberRight;

    private final RelativeEncoder climberLeftEncoder;
    private final RelativeEncoder climberRightEncoder;

    public Climber() {

        // Decalare Motors and get encoders
        climberLeft = new CANSparkMax(ClimberConstants.climberLeftCANID, MotorType.kBrushless);
        climberRight = new CANSparkMax(ClimberConstants.climberRightCANID, MotorType.kBrushless);

        climberLeftEncoder = climberLeft.getEncoder();
        climberRightEncoder = climberRight.getEncoder();

        climberLeft.restoreFactoryDefaults();
        climberRight.restoreFactoryDefaults();

        climberLeft.setIdleMode(IdleMode.kBrake);
        climberRight.setIdleMode(IdleMode.kBrake);

    }

    public void setIndividualRawPower(double powerLeft, double powerRight) {
        climberLeft.set(powerLeft);
        climberRight.set(powerRight);
    }

    public void setClimberLeftPower(double power) {
        climberLeft.set(power);
    }

    public void setClimberRightPower(double power) {
        climberRight.set(power);
    }

    public double[] getEncoderValues() {
        return new double[] {
            climberLeftEncoder.getPosition(),
            climberRightEncoder.getPosition()
        };
    }

}
