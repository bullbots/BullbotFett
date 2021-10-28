// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter_Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Harm;
import frc.robot.subsystems.Shooter;

public class ShootThrottle extends CommandBase {
  /** Creates a new ShootThrottle. */

  private Shooter shooter;
  private Compressor compressor;
  private Harm harm;
  private Timer ball_release_delay;
  private double velocity = 0;
  private DoubleSupplier throttle;
  private boolean servoState = false;

  public ShootThrottle(Shooter shooter, Compressor compressor, Harm harm, DoubleSupplier throttle) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, harm);

    this.shooter = shooter;
    this.compressor = compressor;
    this.harm = harm;
    this.throttle = throttle;
    ball_release_delay = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    compressor.stop();
    ball_release_delay.reset();
    ball_release_delay.start();
    harm.raiseShooterHood();

    velocity = throttle.getAsDouble();

    shooter.set(velocity, -velocity);
    // shooter.ballReleaseServo.set(1);
    // servoState = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!servoState && ball_release_delay.hasElapsed(1)) {
      shooter.ballReleaseServo.set(0);
      servoState = true;
    }
    // SmartDashboard.putNumber("Shooter Velocity from Throttle", velocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.set(0, 0);
    compressor.start();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
