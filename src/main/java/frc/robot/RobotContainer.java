// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;

import frc.robot.Constants.OIConstants;
import frc.robot.commands.Autos;
import frc.robot.StellarController.Button;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.vision.VisionPipeline;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.MechanismSubsystem;
import frc.robot.subsystems.VisionSubsystem;
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

    /*private Tuple2 calc2DDeadband(double valX, double valY, double valDB) {
      private final Tuple2 myVec = new Tuple2<T>(valX, valY);
      return myVec;
    }*/
    
    mechSystem.setDefaultCommand(new RunCommand(() -> {

      int operatorPOV = operatorController.getPOV(0);

      // Intake Pitch Controls
      if (operatorController.getAButtonPressed()) {
        mechSystem.toggleIntakeState();
      }

      // Intake Speed Controls
      if (operatorController.getLeftBumper()) {
        mechSystem.setIntakePower(-1);
      } else if (operatorController.getRightBumper()) {
        mechSystem.setIntakePower(1);
      } else {
        mechSystem.setIntakePower(0);
      }


      // Set Climber Values
      //mechSystem.climber.incramentPosition(MathUtil.applyDeadband(operatorController.getLeftY(), 0.3));


      /* Shooter Speed Controls
      if (operatorController.getXButton()) {
        if (operatorController.getLeftY() > 0.8) {
          mechSystem.incramentShooter(10);
        } else if (operatorController.getLeftY() < -0.8) {
          mechSystem.incramentShooter(-10);
        }
      } else {
        mechSystem.stopShooter();
      }


      // Shooter angle controls
      if (!operatorController.getXButton() && operatorController.getYButton()) {
        if (operatorController.getLeftY() > 0.8) {
          mechSystem.incramentShooterAngle(0.2);
        } else if (operatorController.getLeftY() < -0.8) {
          mechSystem.incramentShooterAngle(-0.2);
        }
      }*/



      // Operator Control Prefrences


      if (operatorController.getXButton()) 
      { // X hold is for shooter preset mode

        // Bumpers touching speaker preset
        if (operatorPOV == 0) 
        {
          mechSystem.setShooterAngle(0);
        }

        // Bumpers touching Amp preset
        if (operatorPOV == 270) 
        {
          mechSystem.setShooterAngle(0);
        }

        // Aligned with chain trap shoot preset
        if (operatorPOV == 180) 
        {
          mechSystem.setShooterAngle(0);
        }

        // Set shooter speed
        if (operatorController.getLeftTriggerAxis() > 0.8)
        {
          //mechSystem.setShooterSpeed(6250);
          mechSystem.setShooterPower(0.5);
        }
        else if (operatorController.getRightTriggerAxis() > 0.8)
        {
          //mechSystem.setShooterSpeed(-6250);
          mechSystem.setShooterPower(-0.5);
        }
        else 
        {
          //mechSystem.setShooterSpeed(0);
          mechSystem.setShooterPower(0);
        }

      } 
      

      else if (operatorController.getYButton()) 
      { // Y hold is designated as a manual shooter control mode

        // Incrament shooter angle in the positive direction
        if (operatorPOV == 0)
        {
          mechSystem.incramentShooterAngle(0.4);
        }

        // Incrament shooter angle in the negative direction
        else if (operatorPOV == 180) 
        {
          mechSystem.incramentShooterAngle(-0.4);
        }

        // Incrament shooter speed
        if (operatorController.getLeftY() > 0.8) 
        {
          mechSystem.incramentShooter(10);
        } 
        else if (operatorController.getLeftY() < -0.8) 
        {
          mechSystem.incramentShooter(-10);
        }


      } 
      

      else if (operatorController.getLeftTriggerAxis() > 0.8 &&
        operatorController.getRightTriggerAxis() > 0.8)
      { // Double bumper hold is designated for climber controls
        
        // Left climber controls
        if (operatorController.getLeftY() > 0.8)
        {
          mechSystem.climber.incramentPositionLeft(0.4);
        }

        else if (operatorController.getLeftY() < -0.8)
        {
          mechSystem.climber.incramentPositionLeft(-0.4);
        }

        // Right climber controls
        if (operatorController.getRightY() > 0.8)
        {
          mechSystem.climber.incramentPositionRight(0.4);
        }

        else if (operatorController.getRightY() < -0.8)
        {
          mechSystem.climber.incramentPositionRight(-0.4);
        }

      } 


      else if (operatorController.getBButton()) 
      { // B hold is a wildcard as of writing this comment
        
      } 
      
      
      else 
      { // Default operating mode that allows intake functionality only

        // Stop the shooter
        mechSystem.setShooterSpeed(0);

        // Set intake power
        if (operatorController.getLeftBumper())
        {
          mechSystem.setIntakePower(1);
        }

        else if (operatorController.getRightBumper())
        {
          mechSystem.setIntakePower(-1);
        }

        else
        {
          mechSystem.setIntakePower(0);
        }

        // Toggle intake angle
        if (operatorController.getAButtonPressed())
        {
          mechSystem.toggleIntakeState();
        }

      }



      // Debugging Code
      /*if (operatorController.getBButton()) {
        if (operatorController.getRightY() < -0.3) {
          mechSystem.climber.setClimberLeftPower(-0.2);
        } else if (operatorController.getRightY() > 0.3) {
          mechSystem.climber.setClimberLeftPower(0.2);
        }
      } else if (operatorController.getYButton()) {
        if (operatorController.getRightY() < -0.3) {
          mechSystem.climber.setClimberRightPower(-0.2);
        } else if (operatorController.getRightY() > 0.3) {
          mechSystem.climber.setClimberRightPower(0.2);
        } else {
          mechSystem.climber.setClimberLeftPower(0);
          mechSystem.climber.setClimberRightPower(0);
        }
      }*/

      

      //mechSystem.setIntakePower(MathUtil.applyDeadband(-operatorController.getLeftTriggerAxis() + operatorController.getRightTriggerAxis(), 0.10));
      //mechSystem.setShooterPower(MathUtil.applyDeadband(operatorController.getLeftY(), 0.20));
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
