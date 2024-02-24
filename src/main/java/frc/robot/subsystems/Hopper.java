package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType; 

public class Hopper {

    private final CANSparkMax hopper;

    public Hopper() {
        hopper = new CANSparkMax(11, MotorType.kBrushless);
    }

    public void setPower(double power) {
        hopper.set(power);
    }
}
