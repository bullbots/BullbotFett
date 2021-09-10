/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Shooter_Commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Harm;
import frc.robot.subsystems.Shooter;

public class ShootDemo extends CommandBase {
  /**
   * Creates a new ShootDemo.
   */

  private Shooter shooter;
  private Compressor compressor;
  private Harm harm;
  private Timer ball_release_delay;
  private double velocity = .25;
  private boolean servoState = false;
  private BooleanSupplier isLongShot;

  public ShootDemo(Shooter shooter, Compressor compressor, Harm harm, BooleanSupplier isLongShot) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, harm);
    this.shooter = shooter;
    this.compressor = compressor;
    this.harm = harm;
    this.isLongShot = isLongShot;
    ball_release_delay = new Timer();

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    compressor.stop();
    ball_release_delay.reset();
    ball_release_delay.start();
    // harm.raiseShooterHood();
    if (isLongShot.getAsBoolean()) {
      harm.raiseShooterHood();
      velocity = .27;
    }else{
      velocity = .90;
    }

    shooter.ballReleaseServo.set(1.0);
    servoState = false;
    shooter.set(velocity-.1, -velocity-.1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!servoState && ball_release_delay.hasElapsed(1)) {
      shooter.ballReleaseServo.set(0);
      servoState = true;
    }
    if (isLongShot.getAsBoolean()) {
      harm.lowerShooterHood();
    // } else {
    //   velocity = 0.32;
    } else {
      harm.raiseShooterHood();
    }
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
