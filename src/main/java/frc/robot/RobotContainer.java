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
import edu.wpi.first.wpilibj.DriverStation;
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
import frc.robot.commands.Autonomous.AutonomousBarrelRace;
import frc.robot.commands.Autonomous.DriveForward;
import frc.robot.commands.Drivetrain_Commands.JoystickDrive;
import frc.robot.commands.Harm_Commands.IntakeBalls;
import frc.robot.commands.Harm_Commands.LowerIntake;
import frc.robot.commands.Harm_Commands.RaiseShooterHood;
import frc.robot.commands.Shooter_Commands.ShootVelocity;
import frc.robot.subsystems.DrivetrainFalcon;
import frc.robot.subsystems.Harm;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
  
  private static JoystickButton button1 = new JoystickButton(stick, 1);
  private static JoystickButton button2 = new JoystickButton(stick, 2);
  private static JoystickButton button3 = new JoystickButton(stick, 3);
  private static JoystickButton button6 = new JoystickButton(stick, 6);
  private static JoystickButton button7 = new JoystickButton(stick, 7);
  private static JoystickButton button11 = new JoystickButton(stick, 11);

  // Subsystems
  private final Shooter shooter = new Shooter();
  private final DrivetrainFalcon drivetrain = new DrivetrainFalcon();
  private final Harm harm = new Harm();

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
    DriverStation.getInstance().silenceJoystickConnectionWarning(true);

    compressor.start();
    
    drivetrain.setDefaultCommand(new JoystickDrive(
      drivetrain,
      () -> -stick.getY(),  // Because Negative Y is forward on the joysticks
      () -> stick.getX(),
      () -> (stick.getZ() - 1)/-2.0
    ));
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
      }
    });

    button2.whileHeld(new ShootVelocity(shooter, harm, () -> !button6.get()));

    button6.whenPressed(new RaiseShooterHood(harm));  // .whenReleased(new LowerShooterHood(harm));

    button7.whileHeld(new LowerIntake(harm));  // .whenReleased(new RaiseIntake(harm));  // Not needed since default command raises Intake, right?
    
    button1.whileHeld(new IntakeBalls(harm));

    button3.whileHeld(new JoystickDrive(
      drivetrain,
      () -> stick.getY(),
      () -> stick.getX(),
      () -> (stick.getZ() - 1) / -2.0
    ));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
//    return new SequentialCommandGroup(
//            new ShootVelocity(shooter, harm, () -> !button6.get()).withTimeout(6),
//            new RunCommand(() -> drivetrain.arcadeDrive(-0.4, 0), drivetrain).withTimeout(3)
//    );
    return new AutonomousBarrelRace(drivetrain, m_trajectory);

    // return new DriveForward(drivetrain).withTimeout(60);
  }

  public void stopAllSubsystems(){
    drivetrain.stop();
  }
  
  public void initializeTrajectory() {
    
    var path_read = new ArrayList<Translation2d>();

    var angle_list = new ArrayList<Double>();

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

          Double tangent_x = Double.parseDouble(sections[2]);
          Double tangent_y = Double.parseDouble(sections[3]);

          Double angle = Math.atan(tangent_y/tangent_x);

          if (Math.signum(tangent_x) == -1.0) {
            angle += 180;
          }
          if (Math.signum(tangent_y) == -1.0) {
            angle += 90;
          }

          angle -= 180; // Rotation2D is bound between -180 and 180, not 0 and 360.

          angle_list.add(angle);

        } catch (NumberFormatException error) {
          System.out.println("Ignore this error:");
          error.printStackTrace();
        }
      }
    } catch (IOException error) {
      System.out.println("Ignore this error:");
      error.printStackTrace();
    }

    // Gets first and last elements and removes them from list.
    double firstX = path_read.get(0).getX();
    double firstY = path_read.get(0).getY();
    double lastX = path_read.get(path_read.size()-1).getX();
    double lastY = path_read.get(path_read.size()-1).getY();

    path_read.remove(0);
    path_read.remove(path_read.size()-1);

    // Gets first and last angle and makes end angle relative to start angle instead of absolute.
    double start_angle = angle_list.get(0);
    double end_angle = angle_list.get(angle_list.size()-1);

    end_angle = end_angle - start_angle;

    m_trajectory =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(firstX, firstY, Rotation2d.fromDegrees(0)),
            path_read,
            new Pose2d(lastX, lastY, Rotation2d.fromDegrees(end_angle)),
            new TrajectoryConfig(3.0, 3.0));

    drivetrain.resetOdometry(m_trajectory.getInitialPose());
  }

  public void periodic() {
    drivetrain.periodic();
  }
}