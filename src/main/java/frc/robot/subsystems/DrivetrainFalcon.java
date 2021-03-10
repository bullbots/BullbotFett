/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.SafeTalonFX;
import frc.robot.util.Shifter;
import frc.robot.util.NavX;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Iterator;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

public class DrivetrainFalcon extends SubsystemBase {

  private double ticks_per_wheel_revolution = 42700.0;

  private double ticks_per_foot = ticks_per_wheel_revolution / (.5 * Math.PI); // .5 is diameter of wheel in feet

  public final SafeTalonFX leftMasterFalcon = new SafeTalonFX(Constants.LEFT_MASTER_PORT, true); // change to false for no PID?
  public final SafeTalonFX rightMasterFalcon = new SafeTalonFX(Constants.RIGHT_MASTER_PORT, true);
  private final SafeTalonFX leftSlaveFalcon = new SafeTalonFX(Constants.LEFT_SLAVE_PORT, true);
  private final SafeTalonFX rightSlaveFalcon = new SafeTalonFX(Constants.RIGHT_SLAVE_PORT, true);

  private final DifferentialDrive diffDrive = new DifferentialDrive(leftMasterFalcon, rightMasterFalcon);
  private final NavX gyro = new NavX();
  public Orchestra orchestra;

  public final Shifter shifter = new Shifter(Constants.LOW_GEAR_CHANNEL, Constants.HIGH_GEAR_CHANNEL);

  private NetworkTableEntry leftCurrent;
  private NetworkTableEntry leftPosition;
  private NetworkTableEntry leftVelocity;

  private NetworkTableEntry rightCurrent;
  private NetworkTableEntry rightPosition;
  private NetworkTableEntry rightVelocity;

  private final double shiftThreshold = 0.8;
  private final double firstGearSlope = 1 / shiftThreshold;
  private final double secondGearSlope = ((21000 - 9240) / (1-shiftThreshold)) / 21000.;
  private final double secondGearIntercept = 26000. / 21000.;

  private double m_leftDist;
  private double m_rightDist;

  private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(gyro.getRotation2d());

  // This gives a maximum value of 40 after 3 seconds of grabbing a value every robot period.
  // List<Double> simulationList = IntStream.rangeClosed(0, 50 * 2).mapToDouble((i)->i * 0.02 * 40.0 / 2.0).boxed().collect(Collectors.toList());
  // Iterator<Double> simIter = simulationList.iterator();

  public DrivetrainFalcon() {
    if (Robot.isReal()) {
      leftSlaveFalcon.follow(leftMasterFalcon);
      rightSlaveFalcon.follow(rightMasterFalcon);

      rightMasterFalcon.setInverted(false);
      rightSlaveFalcon.setInverted(InvertType.FollowMaster);
      leftMasterFalcon.setInverted(true);
      leftSlaveFalcon.setInverted(InvertType.FollowMaster);

      rightMasterFalcon.setNeutralMode(NeutralMode.Coast);
      rightSlaveFalcon.setNeutralMode(NeutralMode.Coast);
      leftMasterFalcon.setNeutralMode(NeutralMode.Coast);
      leftSlaveFalcon.setNeutralMode(NeutralMode.Coast);

      // leftMasterFalcon.configClosedloopRamp(Constants.DRIVETRAIN_RAMP);
      // rightMasterFalcon.configClosedloopRamp(Constants.DRIVETRAIN_RAMP);

      // orchestra = new Orchestra();
      // orchestra.addInstrument(leftMasterFalcon);
      // orchestra.addInstrument(rightMasterFalcon);
      // orchestra.addInstrument(leftSlaveFalcon);
      // orchestra.addInstrument(rightSlaveFalcon);

      // orchestra.loadMusic("test.chrp");

      shifter.shiftLow();
    }

    diffDrive.setRightSideInverted(false);
    diffDrive.setSafetyEnabled(false);

    configurePID();
    // configureMotionMagic();
    configureSmartDashboard();
  }

  public void updateOdometry() {

    m_leftDist = leftMasterFalcon.getSelectedSensorPosition() / ticks_per_foot;
    m_rightDist = rightMasterFalcon.getSelectedSensorPosition() / ticks_per_foot;

    SmartDashboard.putNumber("Left Distance", m_leftDist);
    SmartDashboard.putNumber("Right Distance", m_rightDist);

    m_odometry.update(
        gyro.getRotation2d(), m_leftDist, m_rightDist);
  }

  public void resetOdometry(Pose2d pose) {
    leftMasterFalcon.setSelectedSensorPosition(0);
    rightMasterFalcon.setSelectedSensorPosition(0);
    // m_drivetrainSimulator.setPose(pose);
    m_odometry.resetPosition(pose, gyro.getRotation2d());
  }

  public void resetGyro() {
    gyro.reset();
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  private void configurePID() {
    // Set Velocity PID Constants in slot 0
    leftMasterFalcon.config_kF(0, Constants.LEFT_VELOCITY_FF);
    leftMasterFalcon.config_kP(0, Constants.LEFT_VELOCITY_P);
    leftMasterFalcon.config_kI(0, Constants.LEFT_VELOCITY_I);
    leftMasterFalcon.config_kP(0, Constants.LEFT_VELOCITY_D);

    rightMasterFalcon.config_kF(0, Constants.RIGHT_VELOCITY_FF);
    rightMasterFalcon.config_kP(0, Constants.RIGHT_VELOCITY_P);
    rightMasterFalcon.config_kI(0, Constants.RIGHT_VELOCITY_I);
    rightMasterFalcon.config_kD(0, Constants.RIGHT_VELOCITY_D);

    // leftSlaveFalcon.config_kF(0, Constants.LEFT_VELOCITY_FF);
    // leftSlaveFalcon.config_kP(0, Constants.LEFT_VELOCITY_P);
    // leftSlaveFalcon.config_kI(0, Constants.LEFT_VELOCITY_I);
    // leftSlaveFalcon.config_kP(0, Constants.LEFT_VELOCITY_D);

    // rightSlaveFalcon.config_kF(0, Constants.RIGHT_VELOCITY_FF);
    // rightSlaveFalcon.config_kP(0, Constants.RIGHT_VELOCITY_P);
    // rightSlaveFalcon.config_kI(0, Constants.RIGHT_VELOCITY_I);
    // rightSlaveFalcon.config_kD(0, Constants.RIGHT_VELOCITY_D);
  }

  private void configureMotionMagic() {
    leftMasterFalcon.configMotionCruiseVelocity(Constants.LEFT_MASTER_VELOCITY, Constants.kTIMEOUT_MS);
    leftMasterFalcon.configMotionAcceleration(Constants.LEFT_MASTER_ACCELERATION, Constants.kTIMEOUT_MS);
    
    rightMasterFalcon.configMotionCruiseVelocity(Constants.RIGHT_MASTER_VELOCITY, Constants.kTIMEOUT_MS);
    rightMasterFalcon.configMotionAcceleration(Constants.RIGHT_MASTER_ACCELERATION, Constants.kTIMEOUT_MS);
  }

  private void configureSmartDashboard() {
    leftCurrent = generateEntry("Left Current", 0, 0);
    leftPosition = generateEntry("Left Position", 2, 0);
    leftVelocity = generateEntry("Left Velocity", 4, 0);
    rightCurrent = generateEntry("Right Current", 0, 2);
    rightPosition = generateEntry("Right Position", 2, 2);
    rightVelocity = generateEntry("Right Velocity", 4, 2);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Encoder Ticks - Left", leftMasterFalcon.getSelectedSensorPosition());
    SmartDashboard.putNumber("Encoder Ticks - Right", rightMasterFalcon.getSelectedSensorPosition());
    SmartDashboard.putNumber("Encoder Rate (Normalized) - Left", (leftMasterFalcon.getSelectedSensorVelocity()/ticks_per_foot*10.0)/Constants.MAX_SPEED_LOW_GEAR);
    SmartDashboard.putNumber("Encoder Rate (Normalized) - Right", (rightMasterFalcon.getSelectedSensorVelocity()/ticks_per_foot*10.0)/Constants.MAX_SPEED_LOW_GEAR);

    SmartDashboard.putNumber("NavX Angle", gyro.getRotation2d().getDegrees());

    updateOdometry();

    if (Robot.isReal()) {

      leftCurrent.setNumber(leftMasterFalcon.getStatorCurrent());
      // leftPosition.setNumber(leftMasterFalcon.getSelectedSensorPosition());
      // leftVelocity.setNumber(leftMasterFalcon.getSelectedSensorVelocity());

      rightCurrent.setNumber(rightMasterFalcon.getStatorCurrent());
      // rightPosition.setNumber(rightMasterFalcon.getSelectedSensorPosition());
      // rightVelocity.setNumber(rightMasterFalcon.getSelectedSensorVelocity());

    } else {
      double curLeftCurrent = 0;
      // if (simIter.hasNext()) {
      //   curLeftCurrent = simIter.next();
      // }

      leftCurrent.setNumber(curLeftCurrent);
      leftPosition.setNumber(0.0);
      leftVelocity.setNumber(0.0);

      rightCurrent.setNumber(0.0);
      rightPosition.setNumber(0.0);
      rightVelocity.setNumber(0.0);
    }
  }

  /**
   * Moves the robot according to joystick input
   * @param speed double
   * @param rotation double
   */
  public void arcadeDrive(double speed, double rotation, boolean squareInputs){

    // if (Math.abs(speed) <= shiftThreshold) {
    //   SmartDashboard.putString("State", "Low");
    //   shifter.shiftLow();
    //   SmartDashboard.putNumber("Before", speed);
    //   speed = firstGearSlope * speed;

    //   SmartDashboard.putNumber("After", speed);
    // } else {
    //   SmartDashboard.putNumber("Before", speed);
    //   SmartDashboard.putString("State", "High");
    //   shifter.shiftHigh();
    //   speed = secondGearSlope * (speed - 1) + secondGearIntercept;
    //   SmartDashboard.putNumber("After", speed);
    // }

    diffDrive.arcadeDrive(speed, rotation, squareInputs);
  }

  public void curvatureDrive(double speed, double rotation, boolean isQuickTurn) {
    speed = Math.abs(speed) <= 0.1? 0: speed;
    rotation = Math.abs(rotation) <= 0.1? 0: rotation;
    
    diffDrive.curvatureDrive(speed, rotation, isQuickTurn);
  }

  /**
   * Sets the encoder values back to zero
   */
  public void resetEncoders(){
    System.out.println("Reset Encoders called");
    leftMasterFalcon.setSelectedSensorPosition(0);
    rightMasterFalcon.setSelectedSensorPosition(0);
  }

  /**
   * 
   * @return double array of positions [left, right]
   */
  public double[] getPositions(){
    return new double[] {leftMasterFalcon.getSelectedSensorPosition(), rightMasterFalcon.getSelectedSensorPosition()};
  }

  /**
   * 
   * @return double array of velocities [left, right]
   */
  public double[] getVelocities() {
    return new double[] {leftMasterFalcon.getSelectedSensorVelocity(), rightMasterFalcon.getSelectedSensorVelocity()};
  }

  /**
   * Sets the left and right motors to a percent output
   * @param leftPercent double
   * @param rightPercent double
   */
  public void set(double leftPercent, double rightPercent){
    leftMasterFalcon.set(leftPercent);
    rightMasterFalcon.set(rightPercent);
  }

  /**
   * Sets the left and right motors to a specified control mode
   * @param controlMode ControlMode
   * @param leftMagnitude double
   * @param rightMagnitude double
   */
  public void set(ControlMode controlMode, double leftMagnitude, double rightMagnitude) {
    leftMasterFalcon.set(controlMode, leftMagnitude);
    rightMasterFalcon.set(controlMode, rightMagnitude);
  }

  /**
   * Immediately stops the drivetrain, only use in emergencies
   */
  public void stop(){
    leftMasterFalcon.stopMotor();
    rightMasterFalcon.stopMotor();
  }

  /**
   * Helper function to generate NetworkTableEntries
   */
  private NetworkTableEntry generateEntry(String entryName, int columnIndex, int rowIndex) {
    return Shuffleboard.getTab("Drivetrain")
            .add(entryName, 0)
            .withSize(2,2)
            .withPosition(columnIndex, rowIndex)
            .withWidget(BuiltInWidgets.kGraph)
            .getEntry();
  }

  public void driveLeft(double val) {
    leftMasterFalcon.set(val);
  }

  public void driveRight(double val) {
    rightMasterFalcon.set(val);
  }
}
