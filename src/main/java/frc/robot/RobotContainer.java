// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.Joystick;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // User Input
  private static Joystick stick = new Joystick(0);
  
  private static JoystickButton trigger = new JoystickButton(stick, 1);

  private static JoystickButton button2 = new JoystickButton(stick, 2);
  private static JoystickButton button6 = new JoystickButton(stick, 6);
  private static JoystickButton button7 = new JoystickButton(stick, 7);
  private static JoystickButton button11 = new JoystickButton(stick, 11);

  private static Joystick button_board = new Joystick(1);

  private static JoystickButton climber_up = new JoystickButton(button_board, 1);
  private static JoystickButton climber_down = new JoystickButton(button_board, 2);
  private static JoystickButton control_panel_height = new JoystickButton(button_board, 3);
  private static JoystickButton control_panel_cw = new JoystickButton(button_board, 4);
  private static JoystickButton control_panel_ccw = new JoystickButton(button_board, 5);
  private static JoystickButton shooter_toggle = new JoystickButton(button_board, 6);
  
  // Subsystems
  private final Shooter shooter = new Shooter();
  private final DrivetrainFalcon drivetrain = new DrivetrainFalcon();
  // private final Intake intake = new Intake();
  private final Climb climb = new Climb();
  private final ControlPanel controlPanel = new ControlPanel();

  private final Compressor compressor = new Compressor();

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */

  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    compressor.start();
    
    drivetrain.setDefaultCommand(new JoystickDrive(
      drivetrain,
      () -> -stick.getY(),  // Because Negative Y is forward on the joysticks
      () -> stick.getX(),
      () -> button6.get()
    ));

    

    shooter.setDefaultCommand(
      new RunCommand(() -> shooter.ballReleaseServo.set(-1), shooter)
    );
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    SmartDashboard.putNumber("TurnSpeed", 0);

    SmartDashboard.putNumber("Speed", 0);
    
    trigger.whileHeld(new ShootVelocity(shooter, () -> !shooter_toggle.get()));

    button2.whileHeld(new AlignShooter(drivetrain));

    climber_up.whileHeld(new ClimbUp(climb));
    climber_down.whileHeld(new ClimbDown(climb));
    // control_panel.toggleWhenPressed();
    // control_panel_cw.whileHeld();
    // control_panel_ccw.whileHeld();

    control_panel_height.whenPressed(
      new InstantCommand(() -> controlPanel.toggle(), controlPanel)
    );

    control_panel_cw.whileHeld(new StartEndCommand(() -> controlPanel.spinClockwise(), () -> controlPanel.stop(), controlPanel));
    control_panel_ccw.whileHeld(new StartEndCommand(() -> controlPanel.spinCounterClockwise(), () -> controlPanel.stop(), controlPanel));

    // button3.whileHeld(new IntakeBalls(intake));

    shooter_toggle.whenPressed(new LowerShooter(shooter));
    shooter_toggle.whenReleased(new RaiseShooter(shooter));

    // button7.whenPressed(new StartEndCommand(
    //     () -> intake.toggle(), () -> intake.set(0), intake
    //   ).withTimeout(2)
    // );

    // button6.whileHeld(new StartEndCommand(() -> intake.set(1), () -> intake.set(0), intake));
    // button6.whenReleased(new StartEndCommand(() -> intake.set(-0.2), () -> intake.set(0), intake).withTimeout(0.5));

    // button3.whenPressed(new StartEndCommand(
    //     () -> System.out.println("Start"), () -> System.out.println("End"), intake
    //   ).withTimeout(3)
    // );
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return new PIDTune(drivetrain);
    // return new RunCommand(() -> drivetrain.arcadeDrive(0.3, 0), drivetrain).withTimeout(3);
    return new SequentialCommandGroup(
      new ShootVelocity(shooter, () -> !shooter_toggle.get()).withTimeout(6),
      new RunCommand(() -> drivetrain.arcadeDrive(-0.4, 0), drivetrain).withTimeout(3)
    );
  }

  public void stopAllSubsystems(){
    drivetrain.stop();
  }
}


/*
This is a 16 wheeler 
|_|______|_|
| | |  | | |
    |  |
    |  |
    |  |
    |  |
    |  |
|_|_|__|_|_|
| |  ||  | |
     ||
     ||
     ||
     || 
|_|__||__|_|
| | |  | | |
    |  |
    |  |
    |  |
    |  |
    |  |
|_|_|__|_|_|
| |      | |
This is #3 in the car series.
Made by Triston Van Wyk
*/ 
