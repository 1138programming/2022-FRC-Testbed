// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

// Subsystems:
import frc.robot.subsystems.NeoBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LinearActuator;
import frc.robot.subsystems.Hang;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Storage;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

// Commands
import frc.robot.commands.Base.DriveWithJoysticks;

import frc.robot.commands.Intake.IntakeStop;
import frc.robot.commands.Intake.IntakeIn;
import frc.robot.commands.Intake.IntakeOut;

import frc.robot.commands.Shooter.Shoot;
import frc.robot.commands.Shooter.ShooterStop;

import frc.robot.commands.Storage.StorageStop;

import frc.robot.commands.Hang.HangStop;

import frc.robot.commands.LinearActuator.LinearActuatorOut;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // private final NeoBase base  = new NeoBase();
  private final Intake intake = new Intake();
  private final LinearActuator linearActuator = new LinearActuator();
  // private final Hang hang = new Hang();
  private final Shooter shooter = new Shooter();
  // private final Storage storage = new Storage();

  // private final DriveWithJoysticks driveWithJoysticks = new DriveWithJoysticks(base);

  private final IntakeIn intakeIn = new IntakeIn(intake);
  private final IntakeOut intakeOut = new IntakeOut(intake);
  private final IntakeStop intakeStop = new IntakeStop(intake);

  private final Shoot shoot = new Shoot(shooter);
  private final ShooterStop shooterStop = new ShooterStop(shooter);

  // private final HangStop hangStop = new HangStop(hang);

  // private final StorageStop storageStop= new StorageStop(storage);

  private final LinearActuatorOut linearActuatorOut = new LinearActuatorOut(linearActuator);

  private static final int KLogitechPort = 0;
  private static final int KXboxPort = 0;  

  //Deadzone
  private static final double KDeadZone = 0.05;

  //Logitech Button Constants
  public static final int KLogitechButtonX = 1;
  public static final int KLogitechButtonA = 2;
  public static final int KLogitechButtonB = 3;
  public static final int KLogitechButtonY = 4;
  public static final int KLogitechLeftBumper = 5; 
  public static final int KLogitechRightBumper = 6;
  public static final int KLogitechLeftTrigger = 7;
  public static final int KLogitechRightTrigger = 8;

  private static final int KLeftYAxis = 1;
  private static final int KRightYAxis = 3;
  private static final int KLeftXAxis = 0;
  private static final int KRightXAxis = 2;

  //Xbox Button Constants
  public static final int KXboxButtonA = 1; 
  public static final int KXboxButtonB = 2;
  public static final int KXboxButtonX = 3;  
  public static final int KXboxButtonY = 4; 
  public static final int KXboxLeftBumper = 5; 
  public static final int KXboxRightBumper = 6; 
  public static final int KXboxSelectButton = 7; 
  public static final int KXboxStartButton = 8; 
  public static final int KXboxLeftTrigger = 9; 
  public static final int KXboxRightTrigger = 10; 

  public static Joystick logitech;
  public static XboxController xbox; 
  public JoystickButton logitechBtnX, logitechBtnA, logitechBtnB, logitechBtnY, logitechBtnLB, logitechBtnRB, logitechBtnLT, logitechBtnRT; //Logitech Button
  public JoystickButton xboxBtnA, xboxBtnB, xboxBtnX, xboxBtnY, xboxBtnLB, xboxBtnRB, xboxBtnStrt, xboxBtnSelect, xboxBtnLT, xboxBtnRT;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // base.setDefaultCommand(driveWithJoysticks);
    linearActuator.setDefaultCommand(linearActuatorOut);
    intake.setDefaultCommand(intakeStop);
    // hang.setDefaultCommand(hangStop);
    shooter.setDefaultCommand(shooterStop);
    // hang.setDefaultCommand(hangStop);

    logitech = new Joystick(KLogitechPort);
    xbox = new XboxController(KXboxPort);
    xboxBtnA = new JoystickButton(xbox, 1);
    xboxBtnB = new JoystickButton(xbox, 2);
    xboxBtnX = new JoystickButton(xbox, 3);
    xboxBtnY = new JoystickButton(xbox, 4);
    xboxBtnLB = new JoystickButton(xbox, 5);
    xboxBtnRB = new JoystickButton(xbox, 6);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    xboxBtnLB.whenHeld(shoot);
    xboxBtnX.whenHeld(intakeIn);
    xboxBtnY.whenHeld(intakeOut);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }

  public double getLogiRightYAxis() {
    final double Y = logitech.getRawAxis(KRightYAxis);
    if (Y > KDeadZone || Y < -KDeadZone)
      return -Y;
    else
      return 0;
  }

  public double getLogiLeftYAxis() {
    final double Y = logitech.getRawAxis(KLeftYAxis);
    if(Y > KDeadZone || Y < -KDeadZone)
      return -Y;
    else 
      return 0; 
  }

  public double getLogiRightXAxis() {
    double X = logitech.getRawAxis(KRightXAxis);
    if (X > KDeadZone || X < -KDeadZone) {
      return -X;
    } else {
      return 0; 
    }
  }

  public double getLogiLeftXAxis() {
    double X = logitech.getRawAxis(KLeftXAxis);
    if (X > KDeadZone || X < -KDeadZone) {
      return -X;
    } else {
      return 0;
    }
  }

  public double getXboxLeftAxis() {
    final double Y = xbox.getRawAxis(KLeftYAxis);
    if(Y > KDeadZone || Y < -KDeadZone)
      return -Y;
    else 
      return 0;
  }

  public double getXboxLeftXAxis() {
    final double X = xbox.getRawAxis(KRightXAxis);
    if(X > KDeadZone || X < -KDeadZone)
      return X;
    else 
      return 0;
  }

  public double getXboxRightXAxis() {
    final double X = xbox.getRawAxis(KRightXAxis);
    if (X > KDeadZone || X < -KDeadZone)
      return -X;
    else
      return 0;
  }

  public double getXboxLeftYAxis() {
    final double Y = xbox.getRawAxis(KLeftYAxis);
    if(Y > KDeadZone || Y < -KDeadZone)
      return -Y;
    else 
      return 0;
  }

  public double getXboxRightYAxis() {
    final double Y = xbox.getRawAxis(KRightYAxis);
    if (Y > KDeadZone || Y < -KDeadZone)
      return -Y;
    else
      return 0;
  }
}
