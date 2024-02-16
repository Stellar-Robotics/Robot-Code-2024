// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Autos;
import frc.robot.StellarController.Button;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.MechanismSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem driveSystem = new DriveSubsystem(new Pose2d(6, 6, new Rotation2d(-1)));
  private final MechanismSubsystem mechSystem = new MechanismSubsystem();
  // The driver's controller
  StellarController driverController = new StellarController(OIConstants.kDriverControllerPort);
  XboxController operatorController = new XboxController(OIConstants.kOperatorControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Control intake using right trigger - left trigger
    // Control shooter using left stick y

    /*mechSystem.setDefaultCommand(new ParallelCommandGroup(new RunCommand(
      () -> {
      //mechSystem.setShooterPower(driverController.getAButton()? 1 : 0);
      //mechSystem.setIntakeAngle(operatorController.getLeftY() > 0.5 ? 0.35 : 0.05);
      //mechSystem.setIntakeAngle(operatorController.getPOV(0) - operatorController.getPOV(180));

      if (operatorController.getPOV(0) > 0) {
        mechSystem.setIntakeAngle(0.05);
      } else if (operatorController.getPOV(180) > 0) {
        mechSystem.setIntakeAngle(0.35);
      }

      //mechSystem.setIntakePower(MathUtil.applyDeadband(-operatorController.getLeftTriggerAxis() + operatorController.getRightTriggerAxis(), 0.10));
      //mechSystem.setShooterPower(MathUtil.applyDeadband(operatorController.getLeftY(), 0.20));
    }, mechSystem
    ), new RunCommand(
      () -> {

      if (operatorController.getLeftY() >= 0.5) {
        mechSystem.setIntakePower(1);
      } else if (operatorController.getLeftY() <= -0.5) {
        mechSystem.setIntakePower(0.5);
      }

    }, mechSystem
    )));*/
    
    mechSystem.setDefaultCommand(new RunCommand(() -> {
      //mechSystem.setShooterPower(driverController.getAButton()? 1 : 0);
      mechSystem.setIntakeAngle(operatorController.getLeftY() > 0.5 ? 0.35 : 0.05);
      //mechSystem.setIntakeAngle(operatorController.getPOV(0) - operatorController.getPOV(180));

      /*if (operatorController.getPOV(0) > 0) {
        mechSystem.setIntakeAngle(0.05);
      } else if (operatorController.getPOV(180) > 0) {
        mechSystem.setIntakeAngle(0.35);
      }

      if (operatorController.getLeftY() >= 0.5) {
        mechSystem.setIntakePower(1);
      } else if (operatorController.getLeftY() <= -0.5) {
        mechSystem.setIntakePower(0.5);
      }*/

      mechSystem.setIntakePower(MathUtil.applyDeadband(-operatorController.getLeftTriggerAxis() + operatorController.getRightTriggerAxis(), 0.10));
      //mechSystem.setShooterPower(MathUtil.applyDeadband(operatorController.getLeftY(), 0.20));
    }, mechSystem));

    // Configure the drive command - if the A button is pressed aim at an AprilTag,
    // otherwise respond to driver inputs to control angle.
    driveSystem.setDefaultCommand(
      new ConditionalCommand(
        new RunCommand(
          () -> driveSystem.driveWithAim(
            -MathUtil.applyDeadband(driverController.getLeftY(), OIConstants.kDriveDeadband),
            MathUtil.applyDeadband(driverController.getLeftX(), OIConstants.kDriveDeadband),
            1, // what AprilTag to target???
            true, true), 
            driveSystem
        ),
        // The left stick controls translation of the robot.
        // Turning is controlled by the right stick.
        new RunCommand(
            () -> driveSystem.driveWithAbsoluteAngle(
                -MathUtil.applyDeadband(driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(driverController.getLeftY(), OIConstants.kDriveDeadband),
                driverController.getRightRotary(),
                true, true),
            driveSystem),

        
        driverController::getAButton
      )
    );
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //new Trigger(m_exampleSubsystem::exampleCondition)
       // .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    /*visionSubsystem.setDefaultCommand(
      new RunCommand(() -> visionSubsystem.getAprilTagZ(1), visionSubsystem)
    );*/
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link StellarController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(driverController, Button.kA.value)
        .whileTrue(new RunCommand(
            () -> driveSystem.setX(),
            driveSystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Autos.driveToStage(driveSystem);
  }
}
