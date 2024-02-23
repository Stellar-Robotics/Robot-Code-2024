// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MechanismSubsystem extends SubsystemBase {

  // Define Mechanisms
  private final Intake intake = new Intake();
  private final Shooter shooter = new Shooter();
  public final Climber climber = new Climber();

  public final PIDController aimBotJr;

  public final VisionSubsystem vision;

  /** Creates a new MechanismSubsystem. */
  public MechanismSubsystem() {
    aimBotJr = new PIDController(0.002, 0, 0);
    vision = new VisionSubsystem();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // DELETE THIS AFTER DEBUGGING!
    SmartDashboard.putNumber("IntakeAngle", intake.getAngleEncoderPos());
    SmartDashboard.putNumber("ShooterAngle", shooter.getAngleEncoderPos());

    SmartDashboard.putNumber("ClimberLeftPosition", climber.getEncoderValues()[0]);
    SmartDashboard.putNumber("ClimberRightPosition", climber.getEncoderValues()[1]);
  }


  // Intake
  public void setIntakePower(double xSpeed) {
    intake.setDriveSpeed(xSpeed);
  }

  public void setIntakeAngle(double angle) {
    intake.setTargetAngle(angle);
  }

  public void toggleIntakeState() {
    intake.toggleState();
  }


  // Shooter Setters
  public void setShooterPower(double shooterSpeed) {
    shooter.setDrivePower(shooterSpeed);
  }

  public void setShooterSpeed(double speedRPMs) {
    shooter.setDriveSpeed(speedRPMs);
  }

  public void incramentShooter(double speedRPMsPerSecond) {
    shooter.incramentDriveSpeed(speedRPMsPerSecond);
  }

  public void incramentShooterAngle(double rotations) {
    shooter.incramentAngle(rotations);
  }

  public void stopShooter() {
    shooter.resetDriveSpeed();
  }

  public void setShooterAngle(double angleRotations) {
    shooter.setTargetAngle(angleRotations);
  }

  public void setShooterAngleWithVision() { // WARNING: DO NOT USE THIS METHOD, IT IS NOT WORKING AND CAN BREAK YOUR ROBOT!
    shooter.setTargetAngle(aimBotJr.calculate(shooter.getAngleEncoderPos(), vision.getAprilTagZ(1)));
  }

  // Climber
  public double[] getClimberEncoderPositions() {
    return climber.getEncoderValues();
  }

}
