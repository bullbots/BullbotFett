/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.Harm_Commands.LowerShooterHood;
import frc.robot.commands.Harm_Commands.RaiseIntake;
import frc.robot.util.SafeSparkMax;

public class Harm extends SubsystemBase {
  /**
   * Creates a new Harm (Hood and Arm).
   */

  private SafeSparkMax intake_belt;
  private SafeSparkMax intake_bar_spinner;
  private DoubleSolenoid intake_solenoid = new DoubleSolenoid(Constants.INTAKE_DOWN, Constants.INTAKE_UP);
  private DoubleSolenoid shooter_solenoid = new DoubleSolenoid(Constants.HIGH_ANGLE_CHANNEL, Constants.LOW_ANGLE_CHANNEL);
  private boolean raised = true;

  public Harm() {
    configureShuffleBoard();

    intake_belt = new SafeSparkMax(Constants.INTAKE_BELT, MotorType.kBrushless);
    intake_bar_spinner = new SafeSparkMax(Constants.INTAKE_BAR, MotorType.kBrushless);

    intake_belt.setIdleMode(IdleMode.kBrake);
    intake_bar_spinner.setIdleMode(IdleMode.kCoast);

    shooter_solenoid.set(Value.kForward);

    setDefaultCommand(new ParallelCommandGroup(
      new RaiseIntake(this),
      new LowerShooterHood(this)
    ));
  }

  /** This sets the intake motor
   * @param val This is a value that sets the motor
   */
  public void set(double spinner, double belt){
    intake_bar_spinner.set(spinner);
    intake_belt.set(belt);
  }
/**This sets the value of the intake wheel
 * @param val
 */

  public void intake() {
    set(0.8, 0.3);
  }

  public void raiseIntakeArm() {
    intake_solenoid.set(Value.kForward);
  }

  public void lowerIntakeArm() {
    // System.out.println("LowerIntakeArm is called");
    intake_solenoid.set(Value.kReverse);
  }

  public void raiseShooterHood() {
    shooter_solenoid.set(Value.kForward);
  }

  public void lowerShooterHood() {
    shooter_solenoid.set(Value.kReverse);
  }

  private void configureShuffleBoard() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // intakeVelocity.setDouble(talon.getSelectedSensorVelocity());
  }

  public void stop(){
    set(0, 0);
  }

  public void toggle() {
    if (raised) {
      intake_solenoid.set(Value.kReverse);
    } else {
      intake_solenoid.set(Value.kForward);
    }
    raised = !raised;
  }
}