/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Harm_Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Harm;

public class IntakeBalls extends CommandBase {
  /**
   * Creates a new IntakeBalls.
   */
  private Harm harm;

  public IntakeBalls(Harm harm) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(harm);
    this.harm = harm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    harm.intake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    harm.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
