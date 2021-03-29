/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Drivetrain_Commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainFalcon;

public class JoystickDrive extends CommandBase {
  
  private DrivetrainFalcon m_drivetrain;
  private DoubleSupplier joyY;
  private DoubleSupplier joyX;
  private DoubleSupplier joyZ;
  private BooleanSupplier useThrottle;

  public JoystickDrive(DrivetrainFalcon drivetrain, DoubleSupplier joyY, DoubleSupplier joyX, BooleanSupplier useThrottle) {
    this(drivetrain, joyY, joyX,  () -> 1.0, useThrottle);
  }

  public JoystickDrive(DrivetrainFalcon drivetrain, DoubleSupplier joyY, DoubleSupplier joyX, DoubleSupplier joyZ, BooleanSupplier useThrottle) {
    m_drivetrain = drivetrain;
    this.joyY = joyY;
    this.joyX = joyX;
    this.joyZ = joyZ;
    this.useThrottle = useThrottle;

    addRequirements(m_drivetrain);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double _joyY = joyY.getAsDouble();
    double _joyX = joyX.getAsDouble();
    double _joyZ = useThrottle.getAsBoolean() ? joyZ.getAsDouble() : 1.0;
    boolean isQuickTurn = true;
    m_drivetrain.curvatureDrive(_joyY * _joyZ, _joyX * _joyZ, isQuickTurn);
    // m_drivetrain.arcadeDrive(joyY.getAsDouble(), 0, true);

    // SmartDashboard.putNumber("JoyX", joyX.getAsDouble());
    // SmartDashboard.putNumber("JoyY", joyY.getAsDouble());
    // SmartDashboard.putNumber("JoyZ", joyZ.getAsDouble());

    // if (joyX.getAsDouble() < -.1) {
    //   m_drivetrain.driveLeft(.5);
    // } else if (joyX.getAsDouble() > .1) {
    //   m_drivetrain.driveRight(.5);
    // } else {
    //   m_drivetrain.driveLeft(0);
    //   m_drivetrain.driveRight(0);
    // }
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
