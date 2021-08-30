/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Drivetrain_Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.IntSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import frc.robot.subsystems.DrivetrainFalcon;

public class TurnDistance extends CommandBase {
  /**
   * Creates a new TurnDistance.
   */
  private DrivetrainFalcon drivetrain;
  private IntSupplier turnDistanceSupplier;
  private int turnDistance = 0;
  private double currentPosition;

  private int allowedError = 100;

  public TurnDistance(DrivetrainFalcon drivetrain, int turnDistance) {
    // Use addRequirements() here to declare subsystem dependencies.
    this(drivetrain, ()->turnDistance);

    addRequirements(drivetrain);
  }

  public TurnDistance(DrivetrainFalcon drivetrain, IntSupplier turnDistanceSupplier) {
    this.drivetrain = drivetrain;
    this.turnDistanceSupplier = turnDistanceSupplier;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.resetEncoders();
    turnDistance = turnDistanceSupplier.getAsInt();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.set(ControlMode.MotionMagic, turnDistance, turnDistance);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.set(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double[] positions = drivetrain.getPositions();

    return Math.abs(positions[0] - turnDistance) <= allowedError && Math.abs(positions[1] - turnDistance) <= allowedError;
  }
}
