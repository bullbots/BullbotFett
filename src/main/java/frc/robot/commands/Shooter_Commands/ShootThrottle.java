// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter_Commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class ShootThrottle extends CommandBase {
  /** Creates a new ShootThrottle. */

  private Shooter shooter;
  private Timer ball_release_delay;
  private double vel = 0;
  private DoubleSupplier throttle;
  private boolean servoState = false;

  public ShootThrottle(Shooter shooter, DoubleSupplier throttle) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);

    this.shooter = shooter;
    this.throttle = throttle;
    ball_release_delay = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ball_release_delay.reset();
    ball_release_delay.start();

    vel = throttle.getAsDouble();

    shooter.set(vel, -vel);
    shooter.ballReleaseServo.set(1);
    servoState = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!servoState && ball_release_delay.hasElapsed(1)) {
      shooter.ballReleaseServo.set(0);
      servoState = true;
    }

    SmartDashboard.putNumber("Shooter Velocity from Throttle", vel);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.set(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
