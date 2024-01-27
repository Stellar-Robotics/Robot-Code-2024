// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MechanismSubsystem extends SubsystemBase {

  // Define Mechanisms
  private final Intake floorIntake = new Intake(58);
  private final Shooter shooter = new Shooter(54);

  /** Creates a new MechanismSubsystem. */
  public MechanismSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setIntakePower(double xSpeed) {
    floorIntake.setSpeed(xSpeed);
  }

  public void setShooterPower(double shooterSpeed) {
    shooter.setSpeed(shooterSpeed);
  }
}
