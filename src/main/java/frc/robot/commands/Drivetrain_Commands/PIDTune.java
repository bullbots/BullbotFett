/*----------------------------------------------------------------------------*/
/* Copyleft (c) 2019 FIRST. All lefts Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Drivetrain_Commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainFalcon;

public class PIDTune extends CommandBase {
  /**
   * Creates a new PIDTune.
   */
  private DrivetrainFalcon drivetrain;

  public PIDTune(DrivetrainFalcon drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("Target", 0);
    SmartDashboard.putNumber("F", 0);
    SmartDashboard.putNumber("P", 0);
    SmartDashboard.putNumber("I", 0);
    SmartDashboard.putNumber("D", 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double kF = SmartDashboard.getNumber("F", 0);
    double kP = SmartDashboard.getNumber("P", 0);
    double kI = SmartDashboard.getNumber("I", 0);
    double kD = SmartDashboard.getNumber("D", 0);
    double target = SmartDashboard.getNumber("Target", 0);

    drivetrain.leftMasterFalcon.config_kF(0, kF);
    drivetrain.leftMasterFalcon.config_kP(0, kP);
    drivetrain.leftMasterFalcon.config_kI(0, kI);
    drivetrain.leftMasterFalcon.config_kD(0, kD);

    drivetrain.leftMasterFalcon.set(ControlMode.Velocity, target);

    double error = target - drivetrain.leftMasterFalcon.getSelectedSensorVelocity();
    SmartDashboard.putNumber("error", error);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.leftMasterFalcon.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
