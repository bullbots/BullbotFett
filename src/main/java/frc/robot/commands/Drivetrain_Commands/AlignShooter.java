/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Drivetrain_Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.subsystems.DrivetrainFalcon;

public class AlignShooter extends CommandBase {
  /**
   * Creates a new AlignShooter.
   */
  private DrivetrainFalcon drivetrain;
  private final double kP = 1. / 160;

  private double decay = -.5;
  private double prev_value = 0;

  private Timer timer = new Timer();

  public AlignShooter(DrivetrainFalcon drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x = SmartDashboard.getNumber("TargetX", -9999);

    double value = x * kP;

    if (x != -9999) {
      MathUtil.clamp(value, -.5, .5);

      SmartDashboard.putNumber("value (Saw Target)", value);
    } else {
      double sign = Math.signum(prev_value);

      value = (MathUtil.clamp(decay * timer.get(), 0, Math.abs(prev_value)) + Math.abs(prev_value))* sign;
      SmartDashboard.putNumber("value (Did not see Target)", value);
    }

    drivetrain.arcadeDrive(0, value, false);
    prev_value = value;
    timer.reset();
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
