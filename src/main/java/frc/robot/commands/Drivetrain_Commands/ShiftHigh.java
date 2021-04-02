package frc.robot.commands.Drivetrain_Commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shifter;

public class ShiftHigh extends CommandBase {
    
    private Shifter shifter;
    private BooleanSupplier isAutomatic;

    public ShiftHigh(Shifter shifter, BooleanSupplier isAutomatic) {
      addRequirements(shifter);
      this.shifter = shifter;
      this.isAutomatic = isAutomatic;
    }
    // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (!isAutomatic.getAsBoolean()) {
      shifter.shiftHigh();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
