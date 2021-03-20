// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainFalcon;
import frc.robot.util.TrajectoryManager;

public class DriveBackwardPath extends CommandBase {
  /** Creates a new DriveBackwardPath. */
  
  private DrivetrainFalcon m_drivetrain;
  private Trajectory m_trajectory;

  private final Timer m_timer = new Timer();

  private final RamseteController m_ramsete = new RamseteController();

  public DriveBackwardPath(DrivetrainFalcon drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

    m_drivetrain = drivetrain;

    m_trajectory = TrajectoryManager.generateTrajectories().get("/FORWARD-DISTANCE");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
    m_drivetrain.resetGyro();
    m_drivetrain.resetOdometry(m_trajectory.getInitialPose());

    m_ramsete.setEnabled(true);

    m_drivetrain.setOdometryDirection(true);  
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

    m_drivetrain.arcadeDrive(-normalized_ramsete_speed, normalized_ramsete_rot, false);

    var t_pose = reference.poseMeters;
    var t_x = t_pose.getX();
    var t_y = t_pose.getY();
    var t_rotation = t_pose.getRotation().getDegrees();

    var a_pose = m_drivetrain.getPose();
    var a_x = a_pose.getX();
    var a_y = a_pose.getY();
    var a_rotation = a_pose.getRotation().getDegrees();

    SmartDashboard.putNumber("Ramsete Speed - Normalized", normalized_ramsete_speed);
    SmartDashboard.putNumber("Ramsete Rot - Normalized", normalized_ramsete_rot);

    SmartDashboard.putNumber("Pose X - Trajectory", t_x);
    SmartDashboard.putNumber("Pose Y - Trajectory", t_y);
    SmartDashboard.putNumber("Pose R - Trajectory", t_rotation);

    SmartDashboard.putNumber("Pose X - Actual", a_x);
    SmartDashboard.putNumber("Pose Y - Actual", a_y);
    SmartDashboard.putNumber("Pose R - Actual", a_rotation);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.setOdometryDirection(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.get() > m_trajectory.getTotalTimeSeconds();
  }
}
