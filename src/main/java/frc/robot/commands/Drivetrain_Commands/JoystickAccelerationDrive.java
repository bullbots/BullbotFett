// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain_Commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainFalcon;

public class JoystickAccelerationDrive extends CommandBase {

  private DrivetrainFalcon m_drivetrain;
  private DoubleSupplier joyY;
  private DoubleSupplier joyX;
  private DoubleSupplier joyZ;
  private BooleanSupplier velocityResetButton;

  private double velX;
  private double velY;

  private long startTime;
  
  /**
   * Driving mode that takes the joystick as inputs for the acceleration instead of as the velocity
   * (This mode is for fun only, use with caution)
   * 
   * @param drivetrain
   * @param joyY
   * @param joyX
   * @param resetButton Button that will reset the velocity to 0 when pressed (keeps velocity at 0 while held)
   */
  public JoystickAccelerationDrive(DrivetrainFalcon drivetrain, DoubleSupplier joyY, DoubleSupplier joyX, BooleanSupplier resetButton) {
    this(drivetrain, joyY, joyX,  () -> 1.0, resetButton);
  }

  /**
   * Driving mode that takes the joystick as inputs for the acceleration instead of as the velocity
   * (This mode is for fun only, use with caution)
   * 
   * @param drivetrain
   * @param joyY
   * @param joyX
   * @param joyZ Throttle that limits the speed of the robot
   * @param resetButton Button that will reset the velocity to 0 when pressed (keeps velocity at 0 while held)
   */
  public JoystickAccelerationDrive(DrivetrainFalcon drivetrain, DoubleSupplier joyY, DoubleSupplier joyX, DoubleSupplier joyZ, BooleanSupplier resetButton) {
    m_drivetrain = drivetrain;
    this.joyY = joyY;
    this.joyX = joyX;
    this.joyZ = joyZ;
    this.velocityResetButton = resetButton;

    this.velX = 0;
    this.velY = 0;

    addRequirements(m_drivetrain);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = System.nanoTime();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double accX = joyX.getAsDouble();
    double accY = joyY.getAsDouble();

    // Calculating time since last call to execute
    // Necessary for calculating the change in the velocity
    long currentTime = System.nanoTime();
    long timeDelta = currentTime - startTime;
    startTime = currentTime;
    double timeDeltaInSeconds = timeDelta / 1000000000.;

    // Adding the acceleration to the velocity
    velX += accX * timeDeltaInSeconds;
    velY += accY * timeDeltaInSeconds;

    // Checking button to reset velocity
    if (velocityResetButton.getAsBoolean()) {
      velX = 0;
      velY = 0;
    }

    // Limiting values to 1
    velX = Math.abs(velX) > 1 ? 1 * Math.signum(velX) : velX;
    velY = Math.abs(velY) > 1 ? 1 * Math.signum(velY) : velY;

    SmartDashboard.putNumber("velX", velX);
    SmartDashboard.putNumber("velY", velY);
    SmartDashboard.putNumber("timeDelta", timeDelta);

    m_drivetrain.arcadeDrive(velX * joyZ.getAsDouble(), velY * joyZ.getAsDouble(), true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
