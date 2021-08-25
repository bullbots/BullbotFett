// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain_Commands;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DrivetrainFalcon;

public class TurnAngle extends PIDCommand {
  /** Creates a new TurnAngle. 
   * 
   * Angle is measured from the front of the robot, going counterclockwise
  */
  private DrivetrainFalcon drivetrain;
  private final double kP = 1.0; // Value needs tuned

  private double decay = -.5; // Value needs tuned
  private double prev_value = 0;

  private Timer timer = new Timer();

  private AlignMotorFeedforward m_ff = new AlignMotorFeedforward(.18, 0.0); // Values need tuned

  public TurnAngle(DrivetrainFalcon drivetrain, PIDController controller, DoubleSupplier measurementSource, double setpointSource, DoubleConsumer useOutput) {
    super(controller, measurementSource, setpointSource, useOutput, drivetrain);
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.resetGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var pidOut = m_controller.calculate(m_measurement.getAsDouble(), m_setpoint.getAsDouble());
    var ffOut = m_ff.calculate(m_measurement.getAsDouble());
    m_useOutput.accept(pidOut + ffOut);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    drivetrain.set(0, 0);
  }

  private class AlignMotorFeedforward extends SimpleMotorFeedforward {

    public AlignMotorFeedforward(double ks, double kv) {
      super(ks, kv);
    }

    public double calculate(double targetX, double notAcceleration) {
      int outputRegion = 1;
      if (Math.abs(drivetrain.getGyroAngle()) <= 2) {
        targetX = 0.0;
        outputRegion = 0;
      }
      var ksOut = ks * -Math.signum(targetX);
      // if (Math.abs(targetX) <= Constants.VISION_INNER_ALIGN_THRESHOLD && 
      //   Math.abs(targetX) <= Constants.VISION_OUTER_ALIGN_THRESHOLD) {
      //   ksOut *= 0.5;
      // }

      SmartDashboard.putNumber("outputRegion", outputRegion);
      
      return ksOut + kv * targetX + ka * notAcceleration;
    }
  }
}
