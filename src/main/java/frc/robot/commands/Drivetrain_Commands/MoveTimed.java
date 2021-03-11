/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Drivetrain_Commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DrivetrainFalcon;

public class MoveTimed extends WaitCommand {
  private DrivetrainFalcon drivetrain;
  /**
   * Creates a new MoveTimed.
   */
  public MoveTimed(DrivetrainFalcon drivetrain, double seconds) {
    super(seconds);
    this.drivetrain = drivetrain;
    addRequirements(this.drivetrain);
  }

  @Override
  public void execute() {
    drivetrain.set(ControlMode.PercentOutput, 1, 1);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.set(0, 0);
  }
}
