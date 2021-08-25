// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain_Commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainFalcon;

public class TurnAngle extends CommandBase {
  /** Creates a new TurnAngle. 
   * 
   * Angle is measured from the front of the robot, going counterclockwise
  */
  private DrivetrainFalcon drivetrain;

  private int allowedError = 2; // Two degrees of error
  private double degrees;
  private double leftVelocity = .2;
  private double rightVelocity = .2;
  public TurnAngle(DrivetrainFalcon drivetrain, double degrees) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    this.degrees = degrees % 360.0;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.resetGyro();

    if (degrees < 180) {
      leftVelocity *= -1;
    } else {
      rightVelocity *= -1;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Updates target angle and resets gyro so it knows to turn back if it overshoots
    degrees -= drivetrain.getGyroAngle();
    drivetrain.resetGyro();
    if (degrees < 180) {
      leftVelocity *= -1;
    } else {
      rightVelocity *= -1;
    }

    drivetrain.set(ControlMode.Velocity, leftVelocity, rightVelocity);    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.set(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(drivetrain.getGyroAngle() - degrees) <= allowedError;
  }
}
