package frc.robot.commands.Drivetrain_Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shifter;

public class ShiftHigh extends CommandBase {
    
    private Shifter shifter;

    public ShiftHigh(Shifter shifter) {
      addRequirements(shifter);
      this.shifter = shifter;
    }
    // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shifter.shiftHigh();
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
