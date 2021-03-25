/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Drivetrain_Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainFalcon;

public class AlignShooter extends CommandBase {
  /**
   * Creates a new AlignShooter.
   */
  private DrivetrainFalcon drivetrain;
  private final double kP = 1. / 500;

  public AlignShooter(DrivetrainFalcon drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Align Shooter was called.\n\n");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x = SmartDashboard.getNumber("TargetX", -9999);

    System.out.println(x);

    if (x != -9999) {
      drivetrain.arcadeDrive(0, kP * x, false);
    } else {
      drivetrain.arcadeDrive(0, 0, false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.arcadeDrive(0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
