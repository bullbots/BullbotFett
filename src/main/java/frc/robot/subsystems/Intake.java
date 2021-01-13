/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.SafeTalonSRX;
import frc.robot.util.SafeVictorSPX;


public class Intake extends SubsystemBase {
  /**
   * Creates a new Intake.
   */
  private SafeVictorSPX intake_motor;
  private SafeTalonSRX intake_windsheild_wiper;
  private boolean raised = true;

  public Intake() {
    configureShuffleBoard();

    intake_motor = new SafeVictorSPX(Constants.INTAKE_PORT);
    intake_windsheild_wiper = new SafeTalonSRX(Constants.INTAKE_RAISE_PORT);
    
  }

  /** This sets the intake motor
   * @param val This is a value that sets the motor
   */
  public void set(double val){
    intake_motor.set(val);
  }
/**This sets the value of the intake wheel
 * @param val
 */

  public void raiseIntakeArm(double speed) {
    intake_windsheild_wiper.set(speed);
  }

  public void lowerIntakeArm(double speed) {
    intake_windsheild_wiper.set(-speed);
  }

  private void configureShuffleBoard() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // intakeVelocity.setDouble(talon.getSelectedSensorVelocity());
  }

  public void stop(){
    intake_motor.stopMotor();
  }

  public void toggle() {
    if (raised) {
      intake_windsheild_wiper.set(.4);
    }else {
      intake_windsheild_wiper.set(-1);
    }
    raised = !raised;
  }
}