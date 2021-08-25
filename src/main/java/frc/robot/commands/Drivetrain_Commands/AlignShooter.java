/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Drivetrain_Commands;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainFalcon;
import frc.robot.subsystems.DrivetrainFalcon.CoastMode;

public class AlignShooter extends PIDCommand {
  /**
   * Creates a new AlignShooter.
   */
  private DrivetrainFalcon drivetrain;
  private final double kP = 1. / 950.0;

  private double decay = -.5;
  private double prev_value = 0;

  private Timer timer = new Timer();

  private int m_buffer = 0;
  private boolean m_debugPrint = false;

  private AlignMotorFeedforward m_ff = new AlignMotorFeedforward(0.18, 0.0);

  public AlignShooter(PIDController controller,
  DoubleSupplier measurementSource,
  double setpointSource,
  DoubleConsumer useOutput,
  DrivetrainFalcon drivetrain) {
    super(controller, measurementSource, setpointSource, useOutput, drivetrain);
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    super.initialize();
    drivetrain.setCoastMode(CoastMode.Brake);
    // System.out.println("Info: line shooter initialize called");
    // timer.reset();
    // timer.start();
  }

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
    drivetrain.setCoastMode(CoastMode.Coast);
  }

  // // Returns true when the command should end.
  // @Override
  // public boolean isFinished() {
  //   return false;
  // }

  private void execute_debug() {
    // Called every time the scheduler runs while the command is scheduled.
    double x = SmartDashboard.getNumber("TargetX", -9999);

    double value = x * kP;

    boolean print_buffer = false;

    if (m_debugPrint && m_buffer % 50 == 0) {
      print_buffer = true;
      m_buffer = 0;
    }

    if (x != -9999) {
      value = MathUtil.clamp(value, -.5, .5);

      SmartDashboard.putNumber("value (Saw Target)", value);
      if (print_buffer) {
        System.out.println(String.format("INFO: Saw target: %f", value));
      }
    } else {
      double sign = Math.signum(prev_value);

      double decay_val = decay * timer.get();
      double abs_prev_val = Math.abs(prev_value);
      value = (MathUtil.clamp(decay_val, -abs_prev_val, 0) + abs_prev_val) * sign;

      SmartDashboard.putNumber("value (Did not see Target)", value);
      if (m_debugPrint && print_buffer) {
        System.out.println(String.format("INFO: Did NOT See target decay: %f", decay_val));
        System.out.println(String.format("INFO: Did NOT See target: %f", value));
      }
    }

    drivetrain.arcadeDrive(0, value, false);
    prev_value = value;
    timer.reset();
    if (m_debugPrint) {
      m_buffer++;
    }
  }

  private class AlignMotorFeedforward extends SimpleMotorFeedforward {

    public AlignMotorFeedforward(double ks, double kv) {
      super(ks, kv);
    }

    public double calculate(double targetX, double notAcceleration) {
      int outputRegion = 1;
      if (Math.abs(targetX) <= Constants.VISION_OUTER_ALIGN_THRESHOLD) {
        targetX = 0.0;
        outputRegion = 0;
      }
      var ksOut = ks * -Math.signum(targetX);
      // if (Math.abs(targetX) <= Constants.VISION_INNER_ALIGN_THRESHOLD && 
      //   Math.abs(targetX) <= Constants.VISION_OUTER_ALIGN_THRESHOLD) {
      //   ksOut *= 0.5;
      // }

      SmartDashboard.putNumber("FF Region", outputRegion);
      
      return ksOut + kv * targetX + ka * notAcceleration;
    }
  }
}
