// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionSubsystem extends SubsystemBase {
  NetworkTableInstance nt;
  NetworkTable table;

  DoubleSubscriber xSub;
  DoubleSubscriber zSub;
  DoubleSubscriber rotSub;

  public VisionSubsystem() {
        nt = NetworkTableInstance.getDefault();
        table = nt.getTable("datatable");

        xSub = table.getDoubleTopic("x").subscribe(0.0);
        zSub = table.getDoubleTopic("z").subscribe(0.0);
        rotSub = table.getDoubleTopic("rot").subscribe(0.0);

        nt.startClient4("robot");
        nt.setServer("localhost"); // where TEAM=190, 294, etc, or use inst.setServer("hostname") or similar
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

  public double getAprilTagX(int tagId) {
    return xSub.get();
  }

  public double getAprilTagZ(int tagId) {
    return zSub.get();
  }

  public double getAprilTagRot(int tagId) {
    return rotSub.get();
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
