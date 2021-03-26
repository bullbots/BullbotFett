// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainFalcon;
import frc.robot.util.TrajectoryManager;

public class AutonomousPathB extends CommandBase {
  /** Creates a new AutonomousPathB. */

  private DrivetrainFalcon m_drivetrain;
  private Trajectory m_trajectory;

  private final Timer m_timer = new Timer();

  private final RamseteController m_ramsete = new RamseteController();

  private enum Color {
    RED,
    BLUE
  }
  
  public AutonomousPathB(DrivetrainFalcon drivetrain, Color color) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

    m_drivetrain = drivetrain;
    if (color == Color.BLUE) {
      m_trajectory = TrajectoryManager.getTrajectories().get("/BLUEPATH-B");
    } else if (color == Color.RED) {
      m_trajectory = TrajectoryManager.getTrajectories().get("/REDPATH-B");
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
    m_drivetrain.resetGyro();
    m_drivetrain.resetOdometry(m_trajectory.getInitialPose());

    m_ramsete.setEnabled(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double elapsed = m_timer.get();
    Trajectory.State reference = m_trajectory.sample(elapsed);
    ChassisSpeeds speeds = m_ramsete.calculate(m_drivetrain.getPose(), reference);

    // var ramsete_speed = speeds.vxMetersPerSecond/Constants.MAX_SPEED_LOW_GEAR;
    var ramsete_speed = speeds.vxMetersPerSecond;
    var ramsete_rot = speeds.omegaRadiansPerSecond;

    var normalized_ramsete_speed = ramsete_speed / Constants.MAX_SPEED_LOW_GEAR;
    var normalized_ramsete_rot = -ramsete_rot / Constants.MAX_ANGULAR_VELOCITY;

    m_drivetrain.arcadeDrive(normalized_ramsete_speed, normalized_ramsete_rot, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
