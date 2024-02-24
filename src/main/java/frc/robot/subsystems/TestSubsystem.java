package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestSubsystem extends SubsystemBase {
    private final Shooter shooter = new Shooter();

    public TestSubsystem() {

    }

    public void test() {
        shooter.setDrivePower(-0.5);
    }
}
