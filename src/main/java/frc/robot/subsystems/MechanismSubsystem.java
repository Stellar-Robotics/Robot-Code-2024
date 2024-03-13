// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.PubSub;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MechanismSubsystem extends SubsystemBase {

  // Define Mechanisms
  public final Intake intake = new Intake();
  private final Shooter shooter;
  public final Climber climber = new Climber();
  public final Hopper hopper = new Hopper();

  public final VisionSubsystem vision;

  /** Creates a new MechanismSubsystem. */
  public MechanismSubsystem(VisionSubsystem vision) {
    this.vision = vision;
    this.shooter = new Shooter(vision);
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

  public void incramentIntakeAngle(double rotations) {
    intake.incramentAngle(rotations);
  }

  public double getIntakePos() {
    return intake.getAngleEncoderPos();
  }


  // Shooter Setters
  public void setShooterPower(double shooterSpeed) {
    shooter.setDrivePower(shooterSpeed);
  }

  public void setShooterAngleFromDistance(double distance) {
    shooter.setAngleFromDistance(distance);
  }

  public void setShooterSpeed(double speedRPMs) {
    shooter.setDriveSpeed(speedRPMs);
  }

  public void incramentShooter(double speedRPMsPerSecond) {
    shooter.incramentDriveSpeed(speedRPMsPerSecond);
  }

  public void incramentShooterAngle(double rotations) {
    shooter.incrementAngle(rotations);
  }



  public void stopShooter() {
    shooter.resetDriveSpeed();
  }

  public void setShooterAngle(double angleRotations) {
    shooter.setTargetAngle(angleRotations);
  }

  public void setShooterAngleWithVision() { // WARNING: MAYBE USE THIS METHOD, IT MAY OR MAY NOT BREAK THE ROBOT?
    shooter.setVisionAngle();
  }

  public void executePreset(double position, double speed) {
    shooter.executePreset(position, speed);
  }

  public double getShooterSpeed() {
    return shooter.getShooterSpeed();
  }

  // Climber
  public double[] getClimberEncoderPositions() {
    return climber.getEncoderValues();
  }

}
