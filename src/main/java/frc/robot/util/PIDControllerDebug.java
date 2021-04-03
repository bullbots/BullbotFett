// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.lang.reflect.Field;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/** Add your docs here. */
public class PIDControllerDebug extends PIDController {

    private double m_kp;
    private double m_ki;
    Field m_totalErrorField;
    private boolean m_prev;

    public PIDControllerDebug(double kp, double ki, double kd) {
        super(kp, ki, kd);
        this.m_kp = kp;
        this.m_ki = ki;

        try {
            m_totalErrorField = PIDController.class.getDeclaredField("m_totalError");
            m_totalErrorField.setAccessible(true);

        } catch (NoSuchFieldException | SecurityException e) {
            e.printStackTrace();
        }

        SmartDashboard.putNumber("PID Position Output", 0.0);
        SmartDashboard.putNumber("PID Integral Output", 0.0);
        SmartDashboard.putNumber("PID output", 0.0);
    }

    public double calculate(double measurement) {
        var output = 0.0;
        if(Math.abs(measurement) <= Constants.VISION_ALIGN_THRESHOLD) {
            if (!m_prev) {
                reset();
            }
            output = super.calculate(measurement);
            m_prev = true;
        } else {
            m_prev = false;
        }

        // m_measurement = measurement;
        // m_prevError = m_positionError;

        // if (m_continuous) {
        // m_positionError =
        // MathUtil.inputModulus(m_setpoint - measurement, m_minimumInput,
        // m_maximumInput);
        // } else {
        // m_positionError = m_setpoint - measurement;
        // }

        // m_velocityError = (m_positionError - m_prevError) / m_period;

        // if (m_ki != 0) {
        // m_totalError =
        // MathUtil.clamp(
        // m_totalError + m_positionError * m_period,
        // m_minimumIntegral / m_ki,
        // m_maximumIntegral / m_ki);
        // }

        // return m_kp * m_positionError + m_ki * m_totalError + m_kd * m_velocityError;

        double totalError = 0;
        try {
            totalError = (double) m_totalErrorField.get(this);
        } catch (IllegalArgumentException | IllegalAccessException e) {
            e.printStackTrace();
        }

        double positionOutput = m_kp * getPositionError();
        double integralOutput = m_ki * totalError;

        SmartDashboard.putNumber("PID Position Output", positionOutput);
        SmartDashboard.putNumber("PID Integral Output", integralOutput);
        SmartDashboard.putNumber("PID output", output);
        return output;
      }
}
