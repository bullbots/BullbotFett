/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

import static frc.robot.Constants.NEO_MAX_RPM;


public class ShooterTest extends CommandBase {
  /**
   * Creates a new ShooterTest.
   */
  private Shooter shooter;
  
  public ShooterTest(Shooter shooter) {
    this.shooter = shooter;
    addRequirements(this.shooter);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double top = SmartDashboard.getNumber("Top Speed", 0);
    double bottom = SmartDashboard.getNumber("Bottom Speed", 0);

    //  Min function followed by a Max clamps the value.
    top = Math.max(-NEO_MAX_RPM, Math.min(NEO_MAX_RPM, top));
    bottom = Math.max(-NEO_MAX_RPM, Math.min(NEO_MAX_RPM, bottom));

    shooter.set(top, bottom);
  }


  @Override
  public void end(boolean interrupted) {
    shooter.set(0, 0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}


/*
This is a limousine

    |______|
    | |  | |
      |  |
      |  |
      |  |
      |  |
      |  |
      |  |
      |  |
      |  |
      |  |
    |_|__|_|
    |      |

    This is car #2 in the series
    Made by Triston Van Wyk
    */