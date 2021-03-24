/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Drivetrain_Commands;

import java.security.Principal;

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
  private final double kP = 1. / 950.0;

  private double decay = -.5;
  private double prev_value = 0;

  private Timer timer = new Timer();

  private int m_buffer = 0;
  private boolean m_debugPrint = false;

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

    boolean print_buffer = false;

    if (m_debugPrint && m_buffer % 50 == 0) {
      print_buffer = true;
      m_buffer = 0;
    }

    if (x != -9999) {
      value = MathUtil.clamp(value, -.5, .5);

      SmartDashboard.putNumber("value (Saw Target)", value);
      if (print_buffer) {
        System.out.println(String.format("INFO: Saw target: %f", value));
      }
    } else {
      double sign = Math.signum(prev_value);

      double decay_val = decay * timer.get();
      double abs_prev_val = Math.abs(prev_value);
      value = (MathUtil.clamp(decay_val, -abs_prev_val, 0) + abs_prev_val) * sign;

      SmartDashboard.putNumber("value (Did not see Target)", value);
      if (m_debugPrint && print_buffer) {
        System.out.println(String.format("INFO: Did NOT See target decay: %f", decay_val));
        System.out.println(String.format("INFO: Did NOT See target: %f", value));
      }
    }

    drivetrain.arcadeDrive(0, value, false);
    prev_value = value;
    timer.reset();
    if (m_debugPrint) {
      m_buffer++;
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
