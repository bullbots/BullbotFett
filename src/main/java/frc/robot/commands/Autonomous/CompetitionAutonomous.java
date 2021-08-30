// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Controller;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drivetrain_Commands.AlignShooter;
import frc.robot.commands.Drivetrain_Commands.MoveDistance;
import frc.robot.commands.Drivetrain_Commands.TurnAngle;
import frc.robot.commands.Drivetrain_Commands.TurnDistance;
import frc.robot.commands.Shooter_Commands.ShootVelocity;
import frc.robot.subsystems.DrivetrainFalcon;
import frc.robot.subsystems.Harm;
import frc.robot.subsystems.Shooter;

public class CompetitionAutonomous extends SequentialCommandGroup {
  /**
   * Creates a new CompAuto. Goal of CompAuto is to align the robot than shoot the
   * robot in the beginning 15 seconds of the match
   * 
   * 
   */
  public CompetitionAutonomous(
    DrivetrainFalcon drivetrain,
    Harm harm,
    PIDController controller,
    DoubleSupplier measurementSource,
    double setpointSource,
    DoubleConsumer useOutput,
    Shooter shooter,
    Compressor compressor,
    BooleanSupplier isLongShot
  ) {
    // Use addRequirements() here to declare subsystem dependencies.

    addCommands(
      // Aligns the robot
      new AlignShooter(controller, measurementSource, setpointSource, useOutput, drivetrain).withTimeout(2), // Runs this for 5 seconds hopefully
      // Shoots the ball after the robot is aligned
      new ShootVelocity(shooter, compressor, harm, isLongShot).withTimeout(5), // TODO don't shoot if target not found
      // Figures out how far the robots needs to move to cross over the line
      new TurnDistance(drivetrain, ()->{
        double i = measurementSource.getAsDouble(); // TOOD Freeze this data point
        // Distance of initiation line to goal vertically at a 90 degree angle
        double j = 10.0; 
        // The degree of the robot to the initiation line
        double k = Math.asin(j / i);
        // How many degrees the robot needs to turn to become perpendicular to the initiation line
        double l = 90.0 - Math.toDegrees(k);
        return (int) Math.round(l);
      })
    );
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
