/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Shooter_Commands;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import edu.wpi.first.wpiutil.math.Pair;
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
   private double backspin = 0;
   private BooleanSupplier isLongShot;
   private boolean servoState = false;
   private List<Pair<Integer, Double>> distanceToPower = new ArrayList<>(
    Arrays.asList(
      new Pair<> (99999, 0.0),
      new Pair<> (9300, .45),
      new Pair<> (7100, .39),
      new Pair<> (5800, .38),
      new Pair<> (4100, .4),
      new Pair<> (2700, .22),
      new Pair<> (0, 0.0),
      new Pair<> (-10000, 0.0)));

  public ShootVelocity(Shooter shooter, Harm harm, BooleanSupplier isLongShot) {
    addRequirements(shooter, harm);
    this.shooter = shooter;
    this.harm = harm;
    this.isLongShot = isLongShot;
    ball_release_delay = new Timer();

    SmartDashboard.putNumber("Shooter Velocity", vel);
    SmartDashboard.putNumber("Backspin Factor", backspin);
    var vels = shooter.getVelocities();
    SmartDashboard.putNumber("Top Vel", vels[0]);
    SmartDashboard.putNumber("Bottom Vel", -vels[1]);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ball_release_delay.reset();
    ball_release_delay.start();
    if (isLongShot.getAsBoolean()) {
      harm.raiseShooterHood();
    //   vel = .6;
    // } else {
    //   vel = 0.32;
    }
    // var vel = SmartDashboard.getNumber("Shooter Velocity", 0);
    // var backspin = SmartDashboard.getNumber("Backspin Factor", 0);
    var backspin = 0.3;
    
    var vel = 0.0;
    var cameraDist = SmartDashboard.getNumber("Distance", 0);
    for(var curDistPower:distanceToPower){
      var curDist = curDistPower.getFirst();
      if(curDist < cameraDist){
        break;
      }
      vel = curDistPower.getSecond();
    }

    SmartDashboard.putNumber("Shooter Velocity", vel);
    SmartDashboard.putNumber("Backspin Factor", backspin);

    vel += vel * backspin;
    double top_vel = vel - vel * backspin / 2;
    double bottom_vel = vel + vel * backspin / 2;
    MathUtil.clamp(top_vel, 0, 1);
    MathUtil.clamp(bottom_vel, 0, 1);
    shooter.set(top_vel, -bottom_vel);
    // shooter.set(0.1, -bottom_vel);

    shooter.ballReleaseServo.set(1.0);
    servoState = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!servoState && ball_release_delay.hasElapsed(1)) {
      shooter.ballReleaseServo.set(0);
      servoState = true;
    }

    var vels = shooter.getVelocities();
    SmartDashboard.putNumber("Top Vel", vels[0]);
    SmartDashboard.putNumber("Bottom Vel", -vels[1]);
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
