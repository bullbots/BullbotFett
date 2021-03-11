/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Harm_Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Harm;

public class RaiseShooterHood extends CommandBase {
  /**
   * Creates a new RaiseHarm.
   */
  Harm harm;
  
  public RaiseShooterHood(Harm harm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.harm = harm;

    addRequirements(harm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    harm.raiseShooterHood();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // Do I need to put an end condition, if so, what?  I remember you saying something about end conditions today, but I don't remember if that was about this
  }
}
