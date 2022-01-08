// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;

/** Add your docs here. */
public class DifferentialDriveDebug extends DifferentialDrive {

    // private double m_quickStopThreshold = kDefaultQuickStopThreshold;
    // private double m_quickStopAlpha = kDefaultQuickStopAlpha;
    private double m_quickStopAccumulator;

    public DifferentialDriveDebug(SpeedController leftMotor, SpeedController rightMotor) {
        super(leftMotor, rightMotor);
    }

    // Run super version but then output math for setting motors.
    @SuppressWarnings("ParameterName")
    public void arcadeDrive(double xSpeed, double zRotation, boolean squareInputs) {
        super.arcadeDrive(xSpeed, zRotation, squareInputs);

        // xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);
        // xSpeed = applyDeadband(xSpeed, m_deadband);

        // zRotation = MathUtil.clamp(zRotation, -1.0, 1.0);
        // zRotation = applyDeadband(zRotation, m_deadband);

        // // Square the inputs (while preserving the sign) to increase fine control
        // // while permitting full power.
        // if (squareInputs) {
        // xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
        // zRotation = Math.copySign(zRotation * zRotation, zRotation);
        // }

        // double leftMotorOutput;
        // double rightMotorOutput;

        // double maxInput = Math.copySign(Math.max(Math.abs(xSpeed), Math.abs(zRotation)), xSpeed);

        // if (xSpeed >= 0.0) {
        // // First quadrant, else second quadrant
        // if (zRotation >= 0.0) {
        //     leftMotorOutput = maxInput;
        //     rightMotorOutput = xSpeed - zRotation;
        // } else {
        //     leftMotorOutput = xSpeed + zRotation;
        //     rightMotorOutput = maxInput;
        // }
        // } else {
        // // Third quadrant, else fourth quadrant
        // if (zRotation >= 0.0) {
        //     leftMotorOutput = xSpeed + zRotation;
        //     rightMotorOutput = maxInput;
        // } else {
        //     leftMotorOutput = maxInput;
        //     rightMotorOutput = xSpeed - zRotation;
        // }
        // }

        // // SmartDashboard.putNumber("Left Motor - ArcadeDrive", MathUtil.clamp(leftMotorOutput, -1.0, 1.0) * m_maxOutput);
        // double rightSideInvertMultiplier = isRightSideInverted() ? -1.0 : 1.0;
        // double maxOutput = m_maxOutput * rightSideInvertMultiplier;
        // // SmartDashboard.putNumber("Right Motor - ArcadeDrive", MathUtil.clamp(rightMotorOutput, -1.0, 1.0) * maxOutput);
    }

    @SuppressWarnings({"ParameterName", "PMD.CyclomaticComplexity"})
    public void curvatureDrive(double xSpeed, double zRotation, boolean isQuickTurn) {
        super.curvatureDrive(xSpeed, zRotation, isQuickTurn);
     
        // xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);
        // xSpeed = applyDeadband(xSpeed, m_deadband);

        // zRotation = MathUtil.clamp(zRotation, -1.0, 1.0);
        // zRotation = applyDeadband(zRotation, m_deadband);

        // double angularPower;
        // boolean overPower;

        // if (isQuickTurn) {
        //     if (Math.abs(xSpeed) < m_quickStopThreshold) {
        //         m_quickStopAccumulator =
        //             (1 - m_quickStopAlpha) * m_quickStopAccumulator
        //                 + m_quickStopAlpha * MathUtil.clamp(zRotation, -1.0, 1.0) * 2;
        //     }
        //     overPower = true;
        //     angularPower = zRotation;
        // } else {
        //     overPower = false;
        //     angularPower = Math.abs(xSpeed) * zRotation - m_quickStopAccumulator;

        //     if (m_quickStopAccumulator > 1) {
        //         m_quickStopAccumulator -= 1;
        //     } else if (m_quickStopAccumulator < -1) {
        //         m_quickStopAccumulator += 1;
        //     } else {
        //         m_quickStopAccumulator = 0.0;
        //     }
        // }

        // double leftMotorOutput = xSpeed + angularPower;
        // double rightMotorOutput = xSpeed - angularPower;

        // // If rotation is overpowered, reduce both outputs to within acceptable range
        // if (overPower) {
        //     if (leftMotorOutput > 1.0) {
        //         rightMotorOutput -= leftMotorOutput - 1.0;
        //         leftMotorOutput = 1.0;
        //     } else if (rightMotorOutput > 1.0) {
        //         leftMotorOutput -= rightMotorOutput - 1.0;
        //         rightMotorOutput = 1.0;
        //     } else if (leftMotorOutput < -1.0) {
        //         rightMotorOutput -= leftMotorOutput + 1.0;
        //         leftMotorOutput = -1.0;
        //     } else if (rightMotorOutput < -1.0) {
        //         leftMotorOutput -= rightMotorOutput + 1.0;
        //         rightMotorOutput = -1.0;
        //     }
        // }

        // // Normalize the wheel speeds
        // double maxMagnitude = Math.max(Math.abs(leftMotorOutput), Math.abs(rightMotorOutput));
        // if (maxMagnitude > 1.0) {
        //     leftMotorOutput /= maxMagnitude;
        //     rightMotorOutput /= maxMagnitude;
        // }

        // SmartDashboard.putNumber("Left Motor - CurvatureDrive", leftMotorOutput * m_maxOutput);
        // double rightSideInvertMultiplier = isRightSideInverted() ? -1.0 : 1.0;
        // SmartDashboard.putNumber("Right Motor - CurvatureDrive", rightMotorOutput * m_maxOutput * rightSideInvertMultiplier);
    }
}
