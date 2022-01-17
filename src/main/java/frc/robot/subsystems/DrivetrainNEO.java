package frc.robot.subsystems;


import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.DifferentialDriveDebug;
import frc.robot.util.NavX;
import frc.robot.util.SafeSparkMax;

public class DrivetrainNEO extends SubsystemBase {

    // Setting the ticks rates for wheel revolution, feet, and per hundred milliseconds
    public double ticks_per_wheel_revolution = 4096;
    private double ticks_per_foot = ticks_per_wheel_revolution / (.5 * Math.PI); // 0.5(ft) is Diameter of the wheel in the front
    private double max_ticks_per_hundred_milliseconds = ticks_per_foot * Constants.MAX_SPEED_LOW_GEAR / 10;

    // Inistilizing Master left and right + Slave left and right motors
    private final SafeSparkMax leftMasterNEO = new SafeSparkMax(Constants.LEFT_MASTER_PORT, MotorType.kBrushless);
    private final SafeSparkMax rightMasterNEO = new SafeSparkMax(Constants.RIGHT_MASTER_PORT, MotorType.kBrushless);

    // These are going to be the slave to the master motors if we have two motors on each side
    // private final SafeSparkMax leftSlaveNEO = new SafeSparkMax(Constants.LEFT_SLAVE_PORT, MotorType.kBrushless);
    // private final SafeSparkMax rightSlaveNEO = new SafeSparkMax(Constants.RIGHT_SLAVE_PORT, MotorType.kBrushless);

    private final DifferentialDriveDebug diffDrive = new DifferentialDriveDebug(leftMasterNEO, rightMasterNEO);
    private final NavX gyro = new NavX();

    private NetworkTableEntry leftCurrent;
    private NetworkTableEntry leftPosition;
    private NetworkTableEntry leftVelocity;

    private NetworkTableEntry rightCurrent;
    private NetworkTableEntry rightPosition;
    private  NetworkTableEntry rightVelocity;

    private final double shiftThreshold = 0.8;
    private final double firstGearSlope = 1 / shiftThreshold;
    private final double secondGearSlope = ((21000 - 9240) / (1 - shiftThreshold)) / 21000.0;
    private final double getSecondIntercept = 26000.0 / 21000.0;

    private double m_leftDist;
    private double m_rightDist;

    private boolean m_flippedOdometry;

    private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(gyro.getRotation2d());

    public static final Field2d m_fieldSim = new Field2d();

    public enum CoastMode {
        Coast, Brake
    }

    public DrivetrainNEO() {
        if (Robot.isReal()) {
            rightMasterNEO.setInverted(false);
            leftMasterNEO.setInverted(false);

            setCoastMode(CoastMode.Coast);

            diffDrive.setDeadband(0.05);
        }

        diffDrive.setRightSideInverted(false);
        diffDrive.setSafetyEnabled(false);

        configureSmartDashboard();

        SmartDashboard.putData("Field", m_fieldSim);
    }

    // Might need to add "else if" to set it to kcoast
    public void setCoastMode(CoastMode coastMode) {
        var neutralMode = IdleMode.kCoast;
        if (coastMode == CoastMode.Brake) {
            neutralMode = IdleMode.kBrake;
        }

        rightMasterNEO.setIdleMode(neutralMode);
        leftMasterNEO.setIdleMode(neutralMode);

    }

    public void setOdometryDirection(boolean invert) {
        m_flippedOdometry = invert;
    }

    public void updateOdometry() {
        m_leftDist = (leftMasterNEO.getEncoder().getPosition() / ticks_per_foot);
        m_rightDist = (rightMasterNEO.getEncoder().getPosition() / ticks_per_foot);

        if (m_flippedOdometry) {
            var temporary = -m_leftDist;
            m_leftDist = -m_rightDist;
            m_rightDist = temporary;
        }

        // Debugging values
        // SmartDashboard.putNumber("Left Distance", m_leftDist);
        // SmartDashboard.putNumber("Right Distance", m_rightDist);

        var rotation2d = gyro.getRotation2d();
        if(m_flippedOdometry) {
            rotation2d.rotateBy(Rotation2d.fromDegrees(180));
        }

        m_odometry.update(rotation2d, m_leftDist, m_rightDist);
    }

    public void resetOdometry(Pose2d pose) {
        leftMasterNEO.getEncoder().setPosition(0);
        rightMasterNEO.getEncoder().setPosition(0);

        m_odometry.resetPosition(pose, gyro.getRotation2d());
    }

    public void resetGyro() {
        gyro.reset();
    }

    public void resetGyro180() { gyro.reset180(); }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public void configurePID() {

        // Set Velocity PID Constants in slot 0
        leftMasterNEO.getPIDController().setFF(Constants.LEFT_VELOCITY_FF);
        leftMasterNEO.getPIDController().setP(Constants.LEFT_VELOCITY_P);
        leftMasterNEO.getPIDController().setI(Constants.LEFT_VELOCITY_I);
        leftMasterNEO.getPIDController().setD(Constants.LEFT_VELOCITY_D);

        rightMasterNEO.getPIDController().setFF(Constants.RIGHT_VELOCITY_FF);
        rightMasterNEO.getPIDController().setP(Constants.RIGHT_VELOCITY_P);
        rightMasterNEO.getPIDController().setI(Constants.RIGHT_VELOCITY_I);
        rightMasterNEO.getPIDController().setD(Constants.RIGHT_VELOCITY_D);

    }

    // For Falcon 500 it is "configureMotionMagic"
    public void configureSmartMotion() {
        leftMasterNEO.getPIDController().setSmartMotionMaxVelocity(Constants.LEFT_MASTER_VELOCITY, Constants.kTIMEOUT_MS);
        leftMasterNEO.getPIDController().setSmartMotionMaxAccel(Constants.LEFT_MASTER_ACCELERATION, Constants.kTIMEOUT_MS);

        rightMasterNEO.getPIDController().setSmartMotionMaxVelocity(Constants.RIGHT_MASTER_VELOCITY, Constants.kTIMEOUT_MS);
        rightMasterNEO.getPIDController().setSmartMotionMaxAccel(Constants.RIGHT_MASTER_ACCELERATION, Constants.kTIMEOUT_MS);
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

        // Debugging code
        // System.out.println("DrivetrainFalcon periodic");
        // SmartDashboard.putNumber("Encoder Ticks - Left", leftMasterFalcon.getSelectedSensorPosition());
        // SmartDashboard.putNumber("Encoder Ticks - Right", rightMasterFalcon.getSelectedSensorPosition());
        // SmartDashboard.putNumber("Encoder Rate (Normalized) - Left", (leftMasterFalcon.getSelectedSensorVelocity()/max_ticks_per_hundred_milliseconds));
        // SmartDashboard.putNumber("Encoder Rate (Normalized) - Right", (rightMasterFalcon.getSelectedSensorVelocity()/max_ticks_per_hundred_milliseconds));

        // SmartDashboard.putNumber("NavX Angle", gyro.getRotation2d().getDegrees());

        // SmartDashboard.putNumber("Right Master Current", rightMasterFalcon.getStatorCurrent());
        // SmartDashboard.putNumber("Right Slave Current", rightSlaveFalcon.getStatorCurrent());
        // SmartDashboard.putNumber("Left Master Current", leftMasterFalcon.getStatorCurrent());
        // SmartDashboard.putNumber("Left Slave Current", leftSlaveFalcon.getStatorCurrent());

        updateOdometry();

        if(Robot.isReal()) {
            leftCurrent.setNumber(leftMasterNEO.getOutputCurrent());

            rightCurrent.setNumber(rightMasterNEO.getOutputCurrent());
        } else {
            double curLeftCurrent = 0;

            leftCurrent.setNumber(curLeftCurrent);
            leftPosition.setNumber(0.0);
            leftVelocity.setNumber(0.0);

            rightCurrent.setNumber(0.0);
            rightPosition.setNumber(0.0);
            rightVelocity.setNumber(0.0);
        }
    }

    /**
     * Moves the robot according to the position of the joystick
     * @param speed double
     * @param rotation double
     * @param squareInputs boolean
     */
    public void arcadeDrive(double speed, double rotation, boolean squareInputs) {
        diffDrive.arcadeDrive(speed, rotation, squareInputs);
    }

    public void curvatureDrive(double speed, double rotation, boolean isQuickTurn) {
        diffDrive.curvatureDrive(speed, rotation, isQuickTurn);
    }

    /**
     *  Sets the Encoder values back to zero
     */
    public void resetEncoders() {
        System.out.println("Reset Encoders called");
        leftMasterNEO.getEncoder().setPosition(0);
        rightMasterNEO.getEncoder().setPosition(0);
    }

    /**
     *  Gets the position of the left and right NEO motors
     * @return double array of positions [left, right]
     */
    public double[] getPositions(){
        return new double[] {leftMasterNEO.getEncoder().getPosition(), rightMasterNEO.getEncoder().getPosition()};
    }

    /**
     *  Sets the left and right motors to a percent output
     * @param leftPercent
     * @param rightPercent
     */
    public void set(double leftPercent, double rightPercent) {
        leftMasterNEO.set(leftPercent);
        rightMasterNEO.set(rightPercent);
    }

    // Immediately stops the drivetrain, only use in emergencies
    public void stop() {
        leftMasterNEO.stopMotor();
        rightMasterNEO.stopMotor();
    }

    // Helper function to generate NetworkTableEntries
    private NetworkTableEntry generateEntry(String entryName, int columnIndex, int rowIndex) {
        return Shuffleboard.getTab("NEO_Drivetrain")
                .add(entryName, 0)
                .withSize(2, 2)
                .withPosition(columnIndex, rowIndex)
                .withWidget(BuiltInWidgets.kGraph)
                .getEntry();
    }

    public void driveLeft(double val) {
        leftMasterNEO.set(val);
    }

    public void driveRight(double val) {
        rightMasterNEO.set(val);
    }

}
