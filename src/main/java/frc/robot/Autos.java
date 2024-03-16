// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.MechanismSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Function;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public final class Autos {
  public static TrajectoryConfig getTrajectoryConfig(boolean reversed) {
    TrajectoryConfig config = new TrajectoryConfig(
      AutoConstants.kMaxSpeedMetersPerSecond,
      AutoConstants.kMaxAccelerationMetersPerSecondSquared)
      .setKinematics(DriveConstants.kDriveKinematics);

    config.setReversed(reversed);

    return config;
  }

  public static ProfiledPIDController getThetaController() {
    ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    return thetaController;
  }

  public static Command getStopCommand(DriveSubsystem drive) {
    return Commands.runOnce(() -> {System.out.println("STOP"); drive.drive(0, 0, 0, false, false);}, drive);
  }

  public static Pose2d getPose(double x, double y, double rotation) {
    return new Pose2d(new Translation2d(x, y), new Rotation2d(rotation));
  }

  /*public static SwerveControllerCommand driveToLocationCommand(Location location, DriveSubsystem drive) {
    return driveToLocationCommand(location.pose, drive);
  }*/

  public static SwerveControllerCommand driveToLocationCommand(Pose2d startPose, Pose2d targetPose, boolean reversed, DriveSubsystem drive) {
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      startPose,
      List.of(),
      targetPose,
      getTrajectoryConfig(reversed)
    );


    return new SwerveControllerCommand(
      trajectory,
      drive::getPose, // Functional interface to feed supplier
      DriveConstants.kDriveKinematics,

      // Position controllers
      new PIDController(AutoConstants.kPXController, 0, 0),
      new PIDController(AutoConstants.kPYController, 0, 0),
      getThetaController(),
      drive::setModuleStates,
      drive
    );
  }

  /*public static Command driveToStage(DriveSubsystem drive) {
    return driveToLocationCommand(Location.STAGE, drive).andThen();
  }*/


  // Shoot preloaded gamepiece auto command
  public static Command shootPreloaded(MechanismSubsystem mechSystem, boolean dontStop) {
    // Start by spinning up the shooter and setting its angle (execute preset)
    //BooleanSupplier upToSpeed = () -> { System.out.println(mechSystem.getShooterSpeed()); return mechSystem.getShooterSpeed() >= 3100; };
    //BooleanSupplier intakeFlat = () -> { return mechSystem.getIntakePos() <= 0.16; };

    BooleanSupplier conditions = () -> {

      System.out.println(mechSystem.getShooterSpeed());
      System.out.println(mechSystem.getIntakePos());

      boolean upToSpeed = mechSystem.getShooterSpeed() >= 2500;
      boolean intakeFlat = mechSystem.getIntakePos() <= 0.205;

      if (upToSpeed && intakeFlat) {
        return true;
      } else {
        return false;
      }

    };

    int runAfter = dontStop ? 1:0;

    return runShooterSpeakerPreset(mechSystem)
      .andThen(intakeAngle(0.18, mechSystem))
      .andThen(Commands.waitUntil(conditions))
      .andThen(intakeAndHopperPower(1, 1, mechSystem))
      .andThen(Commands.waitSeconds(1))
      .andThen(intakeAndHopperPower(runAfter, runAfter, mechSystem))
      .andThen(setShooterProfile(0, dontStop ? 4000 : 0, mechSystem));
      
  }

  public static Command aimAndShootPreloadedFromDistance(double distance, MechanismSubsystem mechSystem, DriveSubsystem drive) {
    return rotateTowardsSpeaker(drive)
      .andThen(aimShooterWithDistance(distance, mechSystem))
      .andThen(intakeAngle(0.18, mechSystem))
      .andThen(Commands.waitSeconds(3))
      .andThen(intakeAndHopperPower(1, 1, mechSystem))
      .andThen(Commands.waitSeconds(3))
      .andThen(intakeAndHopperPower(0, 0, mechSystem))
      .andThen(setShooterProfile(0, 0, mechSystem));
  }

  public static Command leave(DriveSubsystem drive) {
    //return driveToLocationCommand(getPose(2.5, 0, 0), drive);

    
    return resetOdometry(drive)
      .andThen(driveToLocationCommand(getPose(0, 0, 0), getPose(1.5, 0, 0), false, drive))
      .andThen(getStopCommand(drive));
  }

  public static Command resetOdometry(DriveSubsystem drive) {
    return Commands.runOnce(() -> {System.out.println("RESET"); drive.resetOdometry(getPose(0, 0, 0));}, drive);
  }


  // Components of the relative autos
  public static Command runShooterSpeakerPreset(MechanismSubsystem mechSystem) {
    return Commands.runOnce(() -> { mechSystem.executePreset(ShooterConstants.speakerPresetPosition, 4000); }, mechSystem);
  }

  public static Command setShooterProfile(double angle, double speed, MechanismSubsystem mechSystem) {
    return Commands.runOnce(() -> { mechSystem.executePreset(angle, speed);}, mechSystem);
  }

  public static Command aimShooterWithDistance(double distance, MechanismSubsystem mechSystem) {
    return Commands.runOnce(() -> {mechSystem.setShooterAngleFromDistance(distance);}, mechSystem);
  }

  public static Command rotateTowardsSpeaker(DriveSubsystem drive) {
    return Commands.runOnce(() -> {drive.driveWithAim(0, 0, 8, true, true);}, drive);
  }

  public static Command intakeAngle(double angle, MechanismSubsystem mechSystem) {
    return Commands.runOnce(() -> { mechSystem.setIntakeAngle(angle);}, mechSystem);
  }

  public static Command intakePower(double power, MechanismSubsystem mechSystem) {
    return Commands.runOnce(() -> { mechSystem.setIntakePower(power);}, mechSystem);
  }

  public static Command hopperPower(double power, MechanismSubsystem mechSystem) {
    return Commands.runOnce(() -> { mechSystem.hopper.setPower(power);}, mechSystem);
  }

  public static Command intakeAndHopperPower(double powerH, double powerI, MechanismSubsystem mechSystem) {
    return Commands.runOnce(() -> { mechSystem.hopper.setPower(powerH); mechSystem.setIntakePower(powerI);}, mechSystem);
  }

  /*public static Command driveForwardAndShoot(DriveSubsystem drive, MechanismSubsystem mechSystem) {
    return
      new RunCommand(() -> {
        mechSystem.setShooterPower(1);
      }, mechSystem)
      .andThen(driveToLocationCommand(getPose(0, 1, 0), drive))
      .andThen(getStopCommand(drive))
      .andThen(new RunCommand(() -> {drive.driveWithAim(0, 0, 8, true, true);}, drive).repeatedly().withTimeout(4));
  }*/

  public static Command hitAndRun(MechanismSubsystem mechSystem, DriveSubsystem drive) {
    return shootPreloaded(mechSystem, false)
    .andThen(intakeAngle(0.35, mechSystem))
    .andThen(intakePower(1, mechSystem))
    .andThen(leave(drive))
    .andThen(intakePower(0, mechSystem))
    .andThen(intakeAngle(0.06, mechSystem));
  }
  public static Command intakeWithCurrentThreshold(MechanismSubsystem mechSystem) {
    BooleanSupplier hitCurrentThreshold = () -> {
      return mechSystem.intake.getOutputCurrent() > 5;
    };

    return Commands.waitUntil(hitCurrentThreshold).withTimeout(5)
      .andThen(Commands.waitSeconds(0.2))
      .andThen(intakePower(0, mechSystem))
      .andThen(intakeAngle(0.16, mechSystem));
  }

  public static Command grabAndGoCurrentThreshold(MechanismSubsystem mechSystem, DriveSubsystem drive) {
    return intakeAngle(0.35, mechSystem)
    .andThen(intakePower(1, mechSystem))
    .andThen(new ParallelCommandGroup(
      leave(drive),
      Commands.waitSeconds(0.5)
        .andThen(intakeWithCurrentThreshold(mechSystem))
    ));
  }

  public static Command waveTwo(MechanismSubsystem mechSystem, DriveSubsystem drive) {
    return shootPreloaded(mechSystem, true)
    .andThen(grabAndGoCurrentThreshold(mechSystem, drive))
    .andThen(runShooterSpeakerPreset(mechSystem))
    .andThen(new ParallelCommandGroup (
      driveToLocationCommand(getPose(2.2, 0, 0), getPose(0, 0, 0), true, drive),
      Commands.waitSeconds(0.5)
      .andThen(intakeAngle(0.16, mechSystem))
    ))
    //.andThen(driveToLocationCommand(getPose(2.2, 0, 0), getPose(0, 0, 0), true, drive))
    .andThen(getStopCommand(drive))
    .andThen(shootPreloaded(mechSystem, false));

  }

  public static Command waveTwoPointFive(MechanismSubsystem mechSystem, DriveSubsystem drive) {
    return shootPreloaded(mechSystem, true)
    .andThen(grabAndGoCurrentThreshold(mechSystem, drive))
    .andThen(runShooterSpeakerPreset(mechSystem))
    .andThen(driveToLocationCommand(getPose(2.2, 0, 0), getPose(0, 0, 0), true, drive))
    .andThen(getStopCommand(drive))
    .andThen(shootPreloaded(mechSystem, false));

  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
