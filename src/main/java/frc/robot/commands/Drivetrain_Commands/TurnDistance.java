/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Drivetrain_Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.IntSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.Constants;
import frc.robot.commands.Autonomous.CompetitionAutonomous.RightOrLeft;
import frc.robot.subsystems.DrivetrainFalcon;


public class TurnDistance extends CommandBase {
  /**
   * Creates a new TurnDistance.
   * 
   * TODO implement the GRYO system is the only concern
   */
  private DrivetrainFalcon drivetrain;

  private IntSupplier targetDistanceSupplier;
  private double turnDistance;
  private int targetDistance = 0;
  private double currentPosition; // What is this used for?
  private RightOrLeft rightOrLeft;

  private int allowedError = 100;

  public TurnDistance(DrivetrainFalcon drivetrain, int turnDistance) {
    this(drivetrain, () -> turnDistance, RightOrLeft.LEFT);
  }

  public TurnDistance(DrivetrainFalcon drivetrain, IntSupplier targetDistanceSupplier, RightOrLeft rightOrLeft) {
    this.drivetrain = drivetrain;
    this.targetDistanceSupplier = targetDistanceSupplier;
    this.rightOrLeft = rightOrLeft;
    addRequirements(drivetrain);
  }


// Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.resetEncoders();
    targetDistance = targetDistanceSupplier.getAsInt();
    // Finds angle
    double turnAngle = 90.0 - Math.toDegrees(Math.asin(10.0 / targetDistance));
    turnDistance = Constants.WHEEL_TO_ROBOT_CENTER_DISTANCE * turnAngle;
        
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Turns right or left depending on selected option
    if (rightOrLeft == RightOrLeft.LEFT) {
      drivetrain.set(ControlMode.MotionMagic, -turnDistance, turnDistance);
    } else if (rightOrLeft == RightOrLeft.RIGHT) {
      drivetrain.set(ControlMode.MotionMagic, turnDistance, -turnDistance);
    } 
      
    
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.set(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double[] positions = drivetrain.getPositions();

    return Math.abs(positions[0] - turnDistance) <= allowedError && Math.abs(positions[1] - turnDistance) <= allowedError;
  }
}
