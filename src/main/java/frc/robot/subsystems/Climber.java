// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.SafeTalonFX;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private SafeTalonFX climber;

  public Climber() {
    climber = new SafeTalonFX(Constants.CLIMBER_PORT, false);

    climber.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void moveUp() {
    climber.set(.5);
  }

  public void moveDown() {
    climber.set(-.5);
  }

  public void stop() {
    climber.stopMotor();
  }
}
