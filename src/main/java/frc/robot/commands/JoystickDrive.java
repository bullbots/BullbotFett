/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainFalcon;

public class JoystickDrive extends CommandBase {
  
  private DrivetrainFalcon m_drivetrain;
  private DoubleSupplier joyY;
  private DoubleSupplier joyX;
  private BooleanSupplier forceLow;

  public JoystickDrive(DrivetrainFalcon drivetrain, DoubleSupplier joyY, DoubleSupplier joyX, BooleanSupplier forceLow) {
    m_drivetrain = drivetrain;
    this.joyY = joyY;
    this.joyX = joyX;
    this.forceLow = forceLow;

    addRequirements(m_drivetrain);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double x = -joyX.getAsDouble();
    
    m_drivetrain.arcadeDrive(joyY.getAsDouble(), x);
  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrain.set(0,0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
