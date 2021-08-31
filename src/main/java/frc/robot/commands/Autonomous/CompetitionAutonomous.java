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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  public enum RightOrLeft {
    RIGHT,
    LEFT
  }
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
    DoubleSupplier setpointSource,
    DoubleConsumer useOutput,
    Shooter shooter,
    Compressor compressor,
    BooleanSupplier isLongShot,
    RightOrLeft rightOrLeft
  ) {
    // Use addRequirements() here to declare subsystem dependencies.

    addCommands(
      // Aligns the robot
      new AlignShooter(controller, measurementSource, setpointSource, useOutput, drivetrain).withTimeout(2), // Runs this for 5 seconds hopefully
      // Shoots the ball after the robot is aligned
      new ShootVelocity(shooter, compressor, harm, isLongShot).withTimeout(5), // TODO don't shoot if target not found
      new TurnDistance(drivetrain, ()->{ 
        int x = (int) SmartDashboard.getNumber("Distance", -9999);
        if (x == -9999) {
          return 0;
        } 
        return x;
      }, rightOrLeft),
      new MoveDistance(drivetrain, 4)
        
          
        
      
      
      // new MoveDistance(drivetrain, 5)
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
