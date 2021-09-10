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
  DrivetrainFalcon m_drivetrain;
  Harm m_harm;
  PIDController m_controller;
  DoubleSupplier m_measurementSource;
  DoubleSupplier m_setpointSource;
  DoubleConsumer m_useOutput;
  Shooter m_shooter;
  Compressor m_compressor;
  BooleanSupplier m_isLongShot;
  RightOrLeft m_rightOrLeft;
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
    System.out.println("CompetitionAutonomous contructor*********************");
    m_drivetrain = drivetrain;
    m_harm = harm;
    m_controller = controller;
    m_measurementSource = setpointSource;
    m_setpointSource = setpointSource;
    m_useOutput = useOutput;
    m_shooter = shooter;
    m_compressor = compressor;
    m_isLongShot = isLongShot;
    m_rightOrLeft = rightOrLeft;
    System.out.println(m_drivetrain);
    System.out.println(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("CompetitionAutonomous initialize*********************");
    addCommands(
      new MoveDistance(m_drivetrain, 4).withTimeout(5),
      // Aligns the robot
      new AlignShooter(m_controller, m_measurementSource, m_setpointSource, m_useOutput, m_drivetrain).withTimeout(2), // Runs this for 5 seconds hopefully
      // Shoots the ball after the robot is aligned
      new ShootVelocity(m_shooter, m_compressor, m_harm, m_isLongShot).withTimeout(5), // TODO don't shoot if target not found
      new TurnDistance(m_drivetrain, ()-> { 
        int cameraDistance = (int) SmartDashboard.getNumber("Distance", -9999);
        if (cameraDistance == -9999) {
          return 0;
        }
        double angle = Math.asin(cameraDistance * 3.28 / 10);
        int turnArc = (int) (angle * 1.125); // wheelbase radius is 1.125 feet
        return turnArc;
      }, m_rightOrLeft).withTimeout(5)
      
        
          
      
        
      
      
      // new MoveDistance(drivetrain, 5)
    );
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("CompetitionAuto execute*************");
    
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
