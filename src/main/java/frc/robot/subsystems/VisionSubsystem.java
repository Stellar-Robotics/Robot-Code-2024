// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionSubsystem extends SubsystemBase {
  NetworkTableInstance nt;
  NetworkTable table;

  DoubleSubscriber xSub;
  DoubleSubscriber zSub;
  DoubleSubscriber rotSub;
  DoubleArraySubscriber absPoseSub;

  public Trajectory targetTrajectory;

  public VisionSubsystem() {
        nt = NetworkTableInstance.getDefault();
        table = nt.getTable("SmartDashboard");

        xSub = table.getDoubleTopic("x").subscribe(0.0);
        zSub = table.getDoubleTopic("z").subscribe(0.0);
        rotSub = table.getDoubleTopic("rot").subscribe(0.0);
        absPoseSub = table.getDoubleArrayTopic("robotPose").subscribe(new double[0]);

        nt.startClient4("robot");
        nt.setServer("localhost"); // where TEAM=190, 294, etc, or use inst.setServer("hostname") or similar
        System.out.println();
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  public double getAprilTagX(int tagId) throws Exception {
    throw new Exception("Add logic for this method to return the x position of the AprilTag *in the camera frame*!");
  }


  /**
   * A method that gets the robot's absolute pose from visible AprilTags. 
   * 
   * @return the absolute pose of the robot based on visible AprilTags
   */
  public Pose2d getRobotPose() {
    //Index 0 is x, 1 is z, and 2 is the rotation in radians
    double[] poseArray = absPoseSub.get();
    if (poseArray.length <= 0) {
      return null;
    }
    Rotation2d rot = Rotation2d.fromRadians(poseArray[2]);
    return new Pose2d(poseArray[1], poseArray[0], rot);
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
