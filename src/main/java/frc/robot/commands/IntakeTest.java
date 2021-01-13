/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeTest extends CommandBase {
  /**
   * Creates a new IntakeTest.
   */
  private Intake intake;

  public IntakeTest(Intake intake) {
    this.intake = intake;

    addRequirements(intake);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double in = SmartDashboard.getNumber("Intake Speed", 0);
    in = in > 1? 1: in;
    in = in < -1? -1: in;
    // intake.set(in);
  }

  @Override
  public void end(boolean interrupted) {
    // intake.set(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
