// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.sql.Driver;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.StellarController.Button;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.vision.VisionPipeline;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.MechanismSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.utils.MiscUtils;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
  private final DriveSubsystem driveSystem = new DriveSubsystem(new Pose2d(0, 0, new Rotation2d(0)), visionSystem);
  private final MechanismSubsystem mechSystem = new MechanismSubsystem(visionSystem);

  // The driver's controller
  StellarController driverController = new StellarController(OIConstants.kDriverControllerPort);
  XboxController operatorController = new XboxController(OIConstants.kOperatorControllerPort);

  public void lightController() {
    final Spark lightController = new Spark(0);
    lightController.set(-0.57);
  }



  

  //PS4Controller altDriveController = new PS4Controller(3);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    mechSystem.setDefaultCommand(new RunCommand(() -> {
      // Create a quick refrence to the operator POV HAT.
      int operatorPOV = operatorController.getPOV(0);


      // PRIMARY OPERATOR CONTROL BINDINGS
      if (operatorController.getXButton()) 
      { // X hold is for shooter preset mode
        mechSystem.setIntakeAngle(0.18);
        mechSystem.intake.isExtended = true;
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
        //mechSystem.setShooterPower(0);
        mechSystem.executePreset(0, 0);

        mechSystem.hopper.setPower(0);




        if (operatorPOV == 0) 
        { // Bumpers touching speaker preset
          //mechSystem.setShooterAngle(ShooterConstants.speakerPresetPosition);
          mechSystem.executePreset(ShooterConstants.speakerPresetPosition, 3800);
        }


        if (operatorPOV == 270) 
        { // Bumpers touching Amp preset
          //mechSystem.setShooterAngle(ShooterConstants.ampPresetPosition);
          mechSystem.executePreset(ShooterConstants.ampPresetPosition, 1650);
        }


        if (operatorPOV == 180) 
        { // Aligned with chain trap shoot preset
          //mechSystem.setShooterAngle(ShooterConstants.trapPresetPosition);
          mechSystem.executePreset(ShooterConstants.trapPresetPosition, 3800);
        }

        if (operatorPOV == 90)
        { // Touching alliance boundry from inside alliance zone
          //mechSystem.setShooterAngle(ShooterConstants.redLinePresetPosition);
          mechSystem.executePreset(ShooterConstants.speakerPresetPosition, 4500);
        }



        if (operatorController.getRightStickButton()) {
          mechSystem.setShooterAngleWithVision();
        }

        if (operatorController.getRightTriggerAxis() > 0.8) {
          mechSystem.setShooterSpeed(3800);
          mechSystem.setShooterAngleWithVision();
        }





        // Things that once existed on the X preset

        if (operatorController.getRightBumperPressed())
        { // Set intake power
          mechSystem.setIntakePower(1);
          // Set hopper if using preset
          if (operatorPOV != -1) {mechSystem.hopper.setPower(1);} else {mechSystem.hopper.setPower(0);}
          mechSystem.intake.startTime = System.currentTimeMillis();
        }
        
        if (operatorController.getLeftBumperPressed())
        {
          mechSystem.setIntakePower(-1);
          if (operatorPOV == -1) {mechSystem.hopper.setPower(-1);} else {mechSystem.hopper.setPower(0);}
          mechSystem.intake.startTime = System.currentTimeMillis();
        }

        if ((System.currentTimeMillis() - mechSystem.intake.startTime > 50) && 
            mechSystem.intake.getOutputCurrent() > 5 && 
            mechSystem.intake.firstSpike) {

          mechSystem.intake.stopTimer.restart();
          mechSystem.intake.firstSpike = false;
        } else if (mechSystem.intake.getOutputCurrent() < 5) {
          mechSystem.intake.firstSpike = true;
        }
        
        if (operatorController.getRightBumperReleased() || 
            operatorController.getLeftBumperReleased() ||
            ( mechSystem.intake.getOutputCurrent() > 5 && 
              !mechSystem.intake.isExtended && 
              mechSystem.intake.stopTimer.hasElapsed(0.1) &&
              !mechSystem.intake.firstSpike))
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
            MiscUtils.transformRange(-driverController.getLeftX(), 0),
            MiscUtils.transformRange(-driverController.getLeftY(), 0),
            1, // what AprilTag to target???
            true, true);
        } else {
          driveSystem.driveWithAbsoluteAngle(
            MiscUtils.transformRange(-driverController.getLeftX(), 0),
            MiscUtils.transformRange(-driverController.getLeftY(), 0),
            driverController.getRightRotary(),
            true, true);
        }

        if (driverController.getAButtonPressed())
        {
          if (driveSystem.driveState == 0) {
            driveSystem.driveState = 1;
          } else {
            driveSystem.driveState = 0;
          }
        }

        if (driverController.getYButtonPressed())
        {
          if (driveSystem.driveState == 2) {
            driveSystem.driveState = 1;
          } else {
            driveSystem.driveState = 2;
          }
        }
        
        if (driverController.getLeftX() == 0 && driverController.getLeftY() == 0 && driveSystem.driveState != 1) {
          driveSystem.driveState = 1;
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
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Autos.waveTwo(mechSystem, driveSystem); //Autos.driveToStage(driveSystem);
  }
}
