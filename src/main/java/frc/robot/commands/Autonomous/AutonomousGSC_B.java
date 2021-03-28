// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.Harm_Commands.IntakeGroup;
import frc.robot.subsystems.DrivetrainFalcon;
import frc.robot.subsystems.Harm;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonomousGSC_B extends SequentialCommandGroup {
  /** Creates a new AutonomousGSC_B. */
  public AutonomousGSC_B(DrivetrainFalcon drivetrain, Harm harm, BooleanSupplier isLoaded, BooleanSupplier isRed) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new WaitUntilCommand(isLoaded),
      new ParallelCommandGroup(
        new ConditionalCommand( // True is red, false is blue
          new TrajectoryBase(drivetrain, "/REDPATH-B", true, true),
          new TrajectoryBase(drivetrain, "/BLUEPATH-B", true, true),
          isRed
        ),
        new IntakeGroup(harm)
      )
    );
  }
}
