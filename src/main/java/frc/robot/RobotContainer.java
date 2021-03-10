// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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

  // private static Joystick button_board = new Joystick(1);

  // // private static JoystickButton climber_up = new JoystickButton(button_board, 1);
  // // private static JoystickButton climber_down = new JoystickButton(button_board, 2);
  // private static JoystickButton control_panel_height = new JoystickButton(button_board, 3);
  // private static JoystickButton control_panel_cw = new JoystickButton(button_board, 4);
  // private static JoystickButton control_panel_ccw = new JoystickButton(button_board, 5);
  // private static JoystickButton shooter_toggle = new JoystickButton(button_board, 6);
  
  // Subsystems
  // private final Shooter shooter;
  private final DrivetrainFalcon drivetrain = new DrivetrainFalcon();
  // private final Intake intake = new Intake();
  // private final Climb climb = new Climb();
  private final ControlPanel controlPanel = new ControlPanel();

  private final Compressor compressor = new Compressor();

  private Trajectory m_trajectory;

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */

  public RobotContainer() {
    // Configure the button bindings
    
    // initializeTrajectory must come before configureButtonBindings
    initializeTrajectory();
    configureButtonBindings();

    // compressor.start();
    compressor.stop();
    
    drivetrain.setDefaultCommand(new JoystickDrive(
      drivetrain,
      () -> -stick.getY(),  // Because Negative Y is forward on the joysticks
      () -> stick.getX(),
      () -> button6.get()
    ));

    // shooter = new Shooter();
    // shooter.setDefaultCommand(
    //   new RunCommand(() -> shooter.ballReleaseServo.set(-1), shooter)
    // );
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    SmartDashboard.putData(new InstantCommand(
      () -> drivetrain.resetEncoders(),
      drivetrain
    ) {
      @Override
      public void initialize() {
        super.initialize();
        setName("Reset Encoders");
        System.out.println("I got called");
      }
    });
    
    // trigger.whileHeld(new ShootVelocity(shooter, () -> !shooter_toggle.get()));

    button2.whileHeld(new AlignShooter(drivetrain));

    // climber_up.whileHeld(new ClimbUp(climb));
    // climber_down.whileHeld(new ClimbDown(climb));
    // control_panel.toggleWhenPressed();
    // control_panel_cw.whileHeld();
    // control_panel_ccw.whileHeld();

    // control_panel_height.whenPressed(
    //   new InstantCommand(() -> controlPanel.toggle(), controlPanel)
    // );

    // control_panel_cw.whileHeld(new StartEndCommand(() -> controlPanel.spinClockwise(), () -> controlPanel.stop(), controlPanel));
    // control_panel_ccw.whileHeld(new StartEndCommand(() -> controlPanel.spinCounterClockwise(), () -> controlPanel.stop(), controlPanel));

    // button3.whileHeld(new IntakeBalls(intake));

    // shooter_toggle.whenPressed(new LowerShooter(shooter));
    // shooter_toggle.whenReleased(new RaiseShooter(shooter));

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
    
    // return new SequentialCommandGroup(
    //   // new ShootVelocity(shooter, () -> !shooter_toggle.get()).withTimeout(6),
    //   new RunCommand(() -> drivetrain.arcadeDrive(-0.4, 0), drivetrain).withTimeout(3)
    // );

    return new AutonomousBarrelRace(drivetrain, m_trajectory);
  }

  public void stopAllSubsystems(){
    drivetrain.stop();
  }

  
  public void initializeTrajectory() {
    
    var path_read = new ArrayList<Translation2d>();

    BufferedReader br = null;
    try {
      // br = new BufferedReader(new FileReader("./Path-1.path"));  
      br = new BufferedReader(new FileReader(Filesystem.getDeployDirectory() + "/Path-1.path"));
    } catch (FileNotFoundException e) {
      e.printStackTrace();
    }
    String line = "";

    try {
      while ((line = br.readLine()) != null) {
        try {
        String[] sections = line.split(",");
        
        double x = Double.parseDouble(sections[0]);
        double y = -Double.parseDouble(sections[1]);

        path_read.add(new Translation2d(x,y));

      } catch (NumberFormatException error) {
          System.out.println("Ignore this error:");
          error.printStackTrace();
        }
      }
    } catch (IOException error) {
      error.printStackTrace();
    }

    // Gets first and last elements and removes them from list.
    double firstX = path_read.get(0).getX();
    double firstY = path_read.get(0).getY();
    double lastX = path_read.get(path_read.size()-1).getX();
    double lastY = path_read.get(path_read.size()-1).getY();

    path_read.remove(0);
    path_read.remove(path_read.size()-1);

    m_trajectory =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(firstX, firstY, new Rotation2d()),
            path_read,
            new Pose2d(lastX, lastY, Rotation2d.fromDegrees(180)), // Useful if start and end point are the same so autonomous can run repeatedly without setup
            new TrajectoryConfig(3.0, 3.0));

    drivetrain.resetOdometry(m_trajectory.getInitialPose());
  }

  public void periodic() {
    drivetrain.periodic();
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
