/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.SafeTalonSRX;
import frc.robot.Constants;

public class ControlPanel extends SubsystemBase {
  
  private final SafeTalonSRX control_panel_talon;
  private final DoubleSolenoid control_panel_solenoid;

  public ControlPanel() {
    control_panel_talon = new SafeTalonSRX(Constants.CONTROL_PANEL_PORT);
    control_panel_solenoid = new DoubleSolenoid(Constants.CONTROL_PANEL_UP_CHANNEL, Constants.CONTROL_PANEL_DOWN_CHANNEL);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void spinClockwise() {
    if (control_panel_solenoid.get().equals(Value.kForward)) {
      control_panel_talon.set(1);
    }
  }

  public void spinCounterClockwise() {
    if (control_panel_solenoid.get().equals(Value.kForward)) {
      control_panel_talon.set(-1);
    }
  }

  public void raise() {
    control_panel_solenoid.set(Value.kForward);
  }

  public void lower() {
    control_panel_solenoid.set(Value.kReverse);
  }

  public void toggle() {
    if (control_panel_solenoid.get().equals(Value.kForward)) {
      lower();
    }else {
      raise();
    }
  }

  public void stop() {
    control_panel_talon.set(0);
  }
}
