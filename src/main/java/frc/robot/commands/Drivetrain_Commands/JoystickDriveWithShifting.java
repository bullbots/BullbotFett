// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain_Commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainFalcon;
import frc.robot.subsystems.Shifter;

// public class JoystickDriveWithShifting extends JoystickDrive {  // Can't do this because it requires me to call super() as first line of constructor.
public class JoystickDriveWithShifting extends CommandBase {
  // Please check this math
  private double shiftThreshold = .8;
  private double lowGearSlope = 1.0/(Math.sqrt(Constants.GEAR_RATIO_LOW));
  private double highGearSlope = (Math.pow(shiftThreshold/lowGearSlope, 2)) / (Math.pow(shiftThreshold, 2));

  private DrivetrainFalcon drivetrain;
  private Shifter shifter;
  private DoubleSupplier joyY;
  private DoubleSupplier joyX;
  private DoubleSupplier joyZ;
  private BooleanSupplier isAutomatic;
  private BooleanSupplier shifterButton;
  private BooleanSupplier useThrottle;

  private boolean isQuickTurn = true;


  public JoystickDriveWithShifting(DrivetrainFalcon drivetrain, Shifter shifter, DoubleSupplier joyY, DoubleSupplier joyX, BooleanSupplier isAutomatic, BooleanSupplier shifterButton, BooleanSupplier useThrottle) {
    this(drivetrain, shifter, joyY, joyX, () -> 1.0, isAutomatic, shifterButton, useThrottle);
  }

  public JoystickDriveWithShifting(DrivetrainFalcon drivetrain, Shifter shifter, DoubleSupplier joyY, DoubleSupplier joyX, DoubleSupplier joyZ, BooleanSupplier isAutomatic, BooleanSupplier shifterButton, BooleanSupplier useThrottle) {
    addRequirements(drivetrain, shifter);
    this.drivetrain = drivetrain;
    this.shifter = shifter;
    this.joyY = joyY;
    this.joyX = joyX;
    this.joyZ = joyZ;
    this.isAutomatic = isAutomatic;
    this.shifterButton = shifterButton;
    this.useThrottle = useThrottle;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double _joyY = joyY.getAsDouble();
    double _joyX = joyX.getAsDouble();
    double _joyZ = useThrottle.getAsBoolean() ? joyZ.getAsDouble() : 1.0;

    if (isAutomatic.getAsBoolean()) {
      if (Math.abs(_joyY) <= shiftThreshold) {
        shifter.shiftLow();
        _joyY *= lowGearSlope;
      } else if (Math.abs(_joyY) > shiftThreshold) {
        shifter.shiftHigh();
        _joyY *= highGearSlope;
      }
    } else {
      if (shifterButton.getAsBoolean()) {
        shifter.shiftHigh();
      } else {
        shifter.shiftLow();
      }
    }

    drivetrain.curvatureDrive(_joyY * _joyZ, _joyX * _joyZ, isQuickTurn);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.set(0,0);
    shifter.shiftLow();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
