// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.Autonomous.AutonomousBarrelRace;
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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
  
  private static JoystickButton button1 = new JoystickButton(stick, 1);
  private static JoystickButton button2 = new JoystickButton(stick, 2);
  private static JoystickButton button3 = new JoystickButton(stick, 3);
  private static JoystickButton button6 = new JoystickButton(stick, 6);  

  // Subsystems
  private final Shooter shooter = new Shooter();
  private final DrivetrainFalcon drivetrain = new DrivetrainFalcon();
  private final Harm harm = new Harm();

  private final Compressor compressor = new Compressor();

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */

  public RobotContainer() {
    // Configure the button bindings
    
    // initializeTrajectory must come before configureButtonBindings
    configureButtonBindings();
    DriverStation.getInstance().silenceJoystickConnectionWarning(true);

    compressor.start();
    
    drivetrain.setDefaultCommand(new JoystickDrive(
      drivetrain,
      () -> -stick.getY() * (button3.get() ? -1.0 : 1.0),  // Because Negative Y is forward on the joysticks
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

    button6.whileHeld(new RaiseShooterHood(harm));  // .whenReleased(new LowerShooterHood(harm));
    
    button1.whileHeld(new ParallelCommandGroup(
      new SequentialCommandGroup(
        new LowerIntake(harm).withTimeout(.5),
        new IntakeBalls(harm)
      ),
      new JoystickDrive(
        drivetrain,
        () -> -stick.getY() * (button3.get() ? -1.0 : 1.0),  // Because Negative Y is forward on the joysticks
        () -> stick.getX()
      )
    ));

    // button3.whileHeld(new JoystickDrive(
    //   drivetrain,
    //   () -> stick.getY(),
    //   () -> stick.getX(),
    //   () -> (stick.getZ() - 1) / -2.0
    // ));
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
    return new AutonomousBarrelRace(drivetrain);

    // return new DriveForward(drivetrain).withTimeout(60);
  }

  public void stopAllSubsystems(){
    drivetrain.stop();
  }
  
  // public void initializeTrajectory() {

  //   var trajectories = TrajectoryManager.generateTrajectories();

  //   for (var trajectoryName : trajectories.keySet()) {
  //     var trajectory = trajectories.get(trajectoryName);
  //   }
  // }

  public void periodic() {
    drivetrain.periodic();
  }
}