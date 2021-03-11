/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Shooter_Commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Harm;
import frc.robot.subsystems.Shooter;

public class ShootVelocity extends CommandBase {
  /**
   * Creates a new Shoot.
   */

   private Shooter shooter;
   private Harm harm;
   private Timer ball_release_delay;
   private double vel = 0;
   private BooleanSupplier isLongShot;
   private boolean servoState = false;

  public ShootVelocity(Shooter shooter, Harm harm, BooleanSupplier isLongShot) {
    addRequirements(shooter, harm);
    this.shooter = shooter;
    this.harm = harm;
    this.isLongShot = isLongShot;
    ball_release_delay = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ball_release_delay.reset();
    ball_release_delay.start();
    if (isLongShot.getAsBoolean()) {
      vel = .7;
      harm.raiseShooterHood();
    }else {
      vel = 0.32;
      harm.lowerShooterHood();
    }
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
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.set(0, 0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
