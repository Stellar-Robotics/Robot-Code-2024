// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Location;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public final class Autos {

  public static TrajectoryConfig getTrajectoryConfig() {
    TrajectoryConfig config = new TrajectoryConfig(
      AutoConstants.kMaxSpeedMetersPerSecond,
      AutoConstants.kMaxAccelerationMetersPerSecondSquared)
      .setKinematics(DriveConstants.kDriveKinematics);

    return config;
  }

  public static ProfiledPIDController getThetaController() {
    ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    return thetaController;
  }

  public static SwerveControllerCommand driveToLocationCommand(Location location, DriveSubsystem drive) {
    return driveToLocationCommand(location.pose, drive);
  }

  public static SwerveControllerCommand driveToLocationCommand(Pose2d targetPose, DriveSubsystem drive) {
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      drive.getPose(),
      List.of(),
      targetPose,
      getTrajectoryConfig()
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

  public static Command visionAuto(DriveSubsystem driveSubsystem, VisionSubsystem vision) {
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      vision.getPose(),
      List.of(),
      new Pose2d(2, 0, new Rotation2d(0)),
      getTrajectoryConfig()
    );

    return new SwerveControllerCommand(
      trajectory,
      vision::getPose, // Functional interface to feed supplier
      DriveConstants.kDriveKinematics,

      // Position controllers
      new PIDController(AutoConstants.kPXController, 0, 0),
      new PIDController(AutoConstants.kPYController, 0, 0),
      getThetaController(),
      driveSubsystem::setModuleStates,
      driveSubsystem,
      vision
    );
  }

  public static Command driveToStage(DriveSubsystem drive) {
    return driveToLocationCommand(Location.STAGE, drive).andThen(new RunCommand(() -> drive.drive(0, 0, 0, false, false)));
  }

  
  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
