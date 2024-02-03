// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import java.util.List;
import java.util.spi.CurrencyNameProvider;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command exampleAuto(ExampleSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  }

  public static Command visionAuto(DriveSubsystem driveSubsystem, VisionSubsystem vision) {
    TrajectoryConfig config = new TrajectoryConfig(
      AutoConstants.kMaxSpeedMetersPerSecond,
      AutoConstants.kMaxAccelerationMetersPerSecondSquared)
      // Add kinematics to ensure max speed is actually obeyed
      .setKinematics(DriveConstants.kDriveKinematics
    );

    var thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      vision.getPose(),
      List.of(),
      new Pose2d(2, 0, new Rotation2d(0)),
      config
    );

    return new SwerveControllerCommand(
      trajectory,
      vision::getPose, // Functional interface to feed supplier
      DriveConstants.kDriveKinematics,

      // Position controllers
      new PIDController(AutoConstants.kPXController, 0, 0),
      new PIDController(AutoConstants.kPYController, 0, 0),
      thetaController,
      driveSubsystem::setModuleStates,
      driveSubsystem,
      vision
    );
  }

  public static Command baseAuto(DriveSubsystem drivetrain) {

    // Create a vision subsystem
    VisionSubsystem visionSubsystem = new VisionSubsystem();
    Pose2d currentPose = drivetrain.getPose();
    System.out.println(currentPose.getX());
    double xVel = currentPose.getX();

    double zStatus = visionSubsystem.getAprilTagZ(1);

    if (zStatus >= 1.5) {
      xVel += 0.2;
    } else if (zStatus <= 1.3) {
      xVel += -0.2;
    }

    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(currentPose.getX(), 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(/*new Translation2d(0, 0.5), 
        new Translation2d(1, 0.5), 
        new Translation2d(1, -0.5), 
        new Translation2d(0,-0.5)*/),
        // End 3 meters straight aphead of where we started, facing forward
        new Pose2d(xVel, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        drivetrain::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        drivetrain::setModuleStates,
        drivetrain);

    // Reset odometry to the starting pose of the trajectory.
    drivetrain.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> drivetrain.drive(0, 0, 0, false, false));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
