// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/** Add your docs here. */
public class DifferentialDriveDebug extends DifferentialDrive{
    @SuppressWarnings("ParameterName")
    public void arcadeDrive(double xSpeed, double zRotation, boolean squareInputs) {
      if (!m_reported) {
        HAL.report(
            tResourceType.kResourceType_RobotDrive, tInstances.kRobotDrive2_DifferentialArcade, 2);
        m_reported = true;
      }
  
      xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);
      xSpeed = applyDeadband(xSpeed, m_deadband);
  
      zRotation = MathUtil.clamp(zRotation, -1.0, 1.0);
      zRotation = applyDeadband(zRotation, m_deadband);
  
      // Square the inputs (while preserving the sign) to increase fine control
      // while permitting full power.
      if (squareInputs) {
        xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
        zRotation = Math.copySign(zRotation * zRotation, zRotation);
      }
  
      double leftMotorOutput;
      double rightMotorOutput;
  
      double maxInput = Math.copySign(Math.max(Math.abs(xSpeed), Math.abs(zRotation)), xSpeed);
  
      if (xSpeed >= 0.0) {
        // First quadrant, else second quadrant
        if (zRotation >= 0.0) {
          leftMotorOutput = maxInput;
          rightMotorOutput = xSpeed - zRotation;
        } else {
          leftMotorOutput = xSpeed + zRotation;
          rightMotorOutput = maxInput;
        }
      } else {
        // Third quadrant, else fourth quadrant
        if (zRotation >= 0.0) {
          leftMotorOutput = xSpeed + zRotation;
          rightMotorOutput = maxInput;
        } else {
          leftMotorOutput = maxInput;
          rightMotorOutput = xSpeed - zRotation;
        }
      }
  
      m_leftMotor.set(MathUtil.clamp(leftMotorOutput, -1.0, 1.0) * m_maxOutput);
      double maxOutput = m_maxOutput * m_rightSideInvertMultiplier;
      m_rightMotor.set(MathUtil.clamp(rightMotorOutput, -1.0, 1.0) * maxOutput);
  
      feed();
    }  
}
