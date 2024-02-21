// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MechanismSubsystem extends SubsystemBase {

  // Define Mechanisms
  private final Intake intake = new Intake();
  private final Shooter shooter = new Shooter();
  private final Climber climber = new Climber();

  /** Creates a new MechanismSubsystem. */
  public MechanismSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // DELETE THIS AFTER DEBUGGING!
    SmartDashboard.putNumber("IntakeAngle", intake.getAngleEncoderPos());
    SmartDashboard.putNumber("ShooterAngle", shooter.getAngleEncoderPos());

    SmartDashboard.putNumber("ClimberLeftPosition", climber.getEncoderValues()[0]);
    SmartDashboard.putNumber("ClimberRightPosition", climber.getEncoderValues()[1]);
  }


  // Intake Setters
  public void setIntakePower(double xSpeed) {
    intake.setDriveSpeed(xSpeed);
  }

  public void setIntakeAngle(double angle) {
    intake.setTargetAngle(angle);
  }

  // Intake Getters

  // Shooter Setters
  public void setShooterPower(double shooterSpeed) {
    shooter.setDriveSpeed(shooterSpeed);
  }

  public void setShooterAngle(double angleRotations) {
    shooter.setTargetAngle(angleRotations);
  }

  public void toggleIntakeState() {
    intake.toggleState();
  }

  public void setRawClimberPower(double powerLeft, double powerRight) {
    climber.setIndividualRawPower(powerLeft, powerRight);
  }

  public void setLeftClimberpower(double power) {
    climber.setClimberLeftPower(power);
  }

  public void setRightClimberPower(double power) {
    climber.setClimberRightPower(power);
  }

  public double[] getClimberEncoderPositions() {
    return climber.getEncoderValues();
  }

  // Shooter Getters

}
