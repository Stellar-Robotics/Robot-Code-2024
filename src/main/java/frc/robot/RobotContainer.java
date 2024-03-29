// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;

import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.Autos;
import frc.robot.StellarController.Button;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.vision.VisionPipeline;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.MechanismSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.utils.MiscUtils;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import edu.wpi.first.wpilibj2.command.button.Trigger;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final VisionSubsystem visionSystem = new VisionSubsystem();
  private final DriveSubsystem driveSystem = new DriveSubsystem(new Pose2d(6, 6, new Rotation2d(-1)), visionSystem);
  private final MechanismSubsystem mechSystem = new MechanismSubsystem(visionSystem);
  // The driver's controller
  StellarController driverController = new StellarController(OIConstants.kDriverControllerPort);
  XboxController operatorController = new XboxController(OIConstants.kOperatorControllerPort);

  //PS4Controller altDriveController = new PS4Controller(3);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    
    mechSystem.setDefaultCommand(new RunCommand(() -> {
      // Create a quick refrence to the operator POV HAT.
      int operatorPOV = operatorController.getPOV(0);


      // PRIMARY OPERATOR CONTROL BINDINGS
      if (operatorController.getXButton()) 
      { // X hold is for shooter preset mode


        if (operatorPOV == 0) 
        { // Bumpers touching speaker preset
          mechSystem.setShooterAngle(ShooterConstants.speakerPresetPosition);
        }


        if (operatorPOV == 270) 
        { // Bumpers touching Amp preset
          mechSystem.setShooterAngle(ShooterConstants.ampPresetPosition);
        }


        if (operatorPOV == 180) 
        { // Aligned with chain trap shoot preset
          mechSystem.setShooterAngle(ShooterConstants.trapPresetPosition);
        }

        if (operatorPOV == 90)
        { // Touching alliance boundry from inside alliance zone
          mechSystem.setShooterAngle(ShooterConstants.redLinePresetPosition);
        }


        if (operatorController.getRightTriggerAxis() > 0.8)
        { // Set shooter speed
          mechSystem.setShooterSpeed(ShooterConstants.presetRPMs);
          //mechSystem.setShooterPower(0.5);
        }
        else if (operatorController.getLeftTriggerAxis() > 0.8)
        {
          mechSystem.setShooterSpeed(-ShooterConstants.presetRPMs);
          //mechSystem.setShooterPower(-0.5);
        }
        else 
        {
          mechSystem.setShooterSpeed(0);
          //mechSystem.setShooterPower(0);
        }


        if (operatorController.getRightBumper())
        { // Set hopper and intake speed bindings
          mechSystem.setIntakePower(1);
          mechSystem.hopper.setPower(0.3);
        }
        else if (operatorController.getLeftBumper())
        {
          mechSystem.setIntakePower(-1);
          mechSystem.hopper.setPower(-0.3);
        }
        else
        {
          mechSystem.setIntakePower(0);
          mechSystem.hopper.setPower(0);
        }


      } 
      

      else if (operatorController.getYButton()) 
      { // Y hold is designated as a manual control mode


        if (operatorPOV == 0)
        { // Incrament shooter angle in the positive direction
          mechSystem.incramentShooterAngle(1);
        }
        else if (operatorPOV == 180)
        { // Incrament shooter angle in the negative direction
          mechSystem.incramentShooterAngle(-1);
        }


        /*if (operatorController.getLeftY() > 0.8) 
        { // Incrament shooter speed
          mechSystem.incramentShooter(1);
          //mechSystem.setShooterPower(0.1);
        } 
        else if (operatorController.getLeftY() < -0.8) 
        {
          mechSystem.incramentShooter(-1);
          //mechSystem.setShooterPower(-0.1);
        }
        else
        {
          //mechSystem.setShooterPower(0);
        }*/


        if (operatorController.getRightBumper())
        { // Set intake power
          mechSystem.setIntakePower(1);
        }
        else if (operatorController.getLeftBumper())
        {
          mechSystem.setIntakePower(-1);
        }
        else
        {
          mechSystem.setIntakePower(0);
        }


        if (operatorPOV == 270)
        {
          mechSystem.incramentIntakeAngle(0.001);
        }
        else if (operatorPOV == 90)
        {
          mechSystem.incramentIntakeAngle(-0.001);
        }


        /*if (operatorController.getLeftTriggerAxis() > 0.8)
        { // Set hopper power
          mechSystem.hopper.setPower(1);
        }
        else if (operatorController.getRightTriggerAxis() > 0.8)
        {
          mechSystem.hopper.setPower(-1);
        }
        else
        {
          mechSystem.hopper.setPower(0);
        }*/

        mechSystem.hopper.setPower(MiscUtils.clamp(-0.3, 0.3, operatorController.getRightTriggerAxis() - operatorController.getLeftTriggerAxis()));


      } 
      

      else if (operatorController.getRightTriggerAxis() > 0.8 &&
        operatorController.getLeftTriggerAxis() > 0.8)
      { // Double bumper hold is designated for climber controls
        

        if (operatorController.getLeftY() > 0.8)
        { // Left climber controls
          mechSystem.climber.incramentPositionLeft(0.4);
        }
        else if (operatorController.getLeftY() < -0.8)
        {
          mechSystem.climber.incramentPositionLeft(-0.4);
        }
        else
        {
          mechSystem.climber.incramentPositionLeft(0);
        }


        if (operatorController.getRightY() > 0.8)
        { // Right climber controls
          mechSystem.climber.incramentPositionRight(0.4);
        }
        else if (operatorController.getRightY() < -0.8)
        {
          mechSystem.climber.incramentPositionRight(-0.4);
        }
        else
        {
          mechSystem.climber.incramentPositionRight(0);
        }


      } 


      else if (operatorController.getBButtonPressed()) 
      { // B hold is a wildcard as of writing this comment
        mechSystem.setIntakeAngle(0.06);
        mechSystem.intake.isExtended = true;
      } 
      
      
      else 
      { // Default operating mode that allows intake functionality only


        // Stop the shooter
        //mechSystem.setShooterSpeed(0);
        mechSystem.setShooterPower(0);


        if (operatorController.getRightBumper())
        { // Set intake power
          mechSystem.setIntakePower(1);
        }
        else if (operatorController.getLeftBumper())
        {
          mechSystem.setIntakePower(-1);
        }
        else
        {
          mechSystem.setIntakePower(0);
        }


        if (operatorController.getAButtonPressed() || operatorController.getAButtonReleased())
        { // Toggle intake angle
          mechSystem.toggleIntakeState();
        }

        // TMP - Set climbers to zero when not in double bumper mode.
        mechSystem.climber.setClimberLeftPosition(140);
        mechSystem.climber.setClimberRightPosition(140);


      }

    }, mechSystem));


    // Configure the drive command - if the A button is pressed aim at an AprilTag,
    // otherwise respond to driver inputs to control angle.
    driveSystem.setDefaultCommand(
      new RunCommand(() -> {

        // Primairy Stellar Controller
        if (driverController.getRightCenterButton()) {
          driveSystem.driveWithAim(
            -driverController.getLeftX(),
            -driverController.getLeftY(),
            1, // what AprilTag to target???
            true, true);
        } else {
          driveSystem.driveWithAbsoluteAngle(
            -driverController.getLeftX(),
            -driverController.getLeftY(),
            driverController.getRightRotary(),
            true, true);
        }

        // Basic Alt Controller
        /*if (altDriveController.getL1Button()) {
          driveSystem.driveWithAim(
            -MathUtil.applyDeadband(altDriveController.getLeftX(), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(altDriveController.getLeftY(), OIConstants.kDriveDeadband),
            1, // what AprilTag to target???
            true, true);
        } else {
          driveSystem.driveWithJoystick(
            -MathUtil.applyDeadband(altDriveController.getLeftX(), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(altDriveController.getLeftY(), OIConstants.kDriveDeadband),
            altDriveController.getRightX(), altDriveController.getRightY(),
            true, true);
        }*/


      }, driveSystem)
      /*new ConditionalCommand(
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
      ) */
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
  //private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //new Trigger(m_exampleSubsystem::exampleCondition)
       // .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    /*visionSubsystem.setDefaultCommand(
      new RunCommand(() -> visionSubsystem.getAprilTagZ(1), visionSubsystem)
    );*/
  //}

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
