// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain_Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainFalcon;
import frc.robot.subsystems.Shifter;

public class ShiftAutomatic extends CommandBase {
  /** Creates a new ShiftAutomatic. */
  private Shifter shifter;
  private DrivetrainFalcon drivetrain;

  // Taken directly out of DrivetrainFalcon.java
  private double shiftThreshold = .8;
  private final double firstGearSlope = 1 / shiftThreshold;
  private final double secondGearSlope = ((21000 - 9240) / (1-shiftThreshold)) / 21000.;
  private final double secondGearIntercept = 26000. / 21000.;

  public ShiftAutomatic(Shifter shifter, DrivetrainFalcon drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shifter, drivetrain);
    this.shifter = shifter;
    this.drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // THIS IS NOT FUNCTIONAL YET.  REQUIRES INTERACTION WITH DRIVETRAIN, MAY NEED TO RECREATE JOYSTICKDRIVE.
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
