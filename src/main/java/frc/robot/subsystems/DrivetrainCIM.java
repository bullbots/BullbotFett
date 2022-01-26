package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.DifferentialDriveDebug;
import frc.robot.util.NavX;


public class DrivetrainCIM extends SubsystemBase {
    // Inistalizing Master left and right + Slave left and right motors
    private final SpeedController leftMasterCIM = new Talon(Constants.LEFT_MASTER_PORT);
    private final SpeedController rightMasterCIM = new Talon(Constants.RIGHT_MASTER_PORT);

    private final SpeedController leftSlaveCIM = new Talon(Constants.LEFT_SLAVE_PORT);
    private final SpeedController rightSlaveCIM = new Talon(Constants.RIGHT_SLAVE_PORT);

    private final DifferentialDriveDebug diffDrive = new DifferentialDriveDebug(leftMasterCIM, rightMasterCIM);
    private final NavX gyro = new NavX();

    private NetworkTableEntry leftCurrent;
    private NetworkTableEntry leftPosition;
    private NetworkTableEntry leftVelocity;

    private NetworkTableEntry rightCurrent;
    private NetworkTableEntry rightPosition;
    private NetworkTableEntry rightVelocity;

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

    public DrivetrainCIM() {
        if (Robot.isReal()) {
            rightMasterCIM.setInverted(false);
            leftMasterCIM.setInverted(false);


            diffDrive.setDeadband(0.05);
        }

        diffDrive.setRightSideInverted(false);
        diffDrive.setSafetyEnabled(false);

        configureSmartDashboard();

        SmartDashboard.putData("Field", m_fieldSim);
    }


    public void setOdometryDirection(boolean invert) {
        m_flippedOdometry = invert;
    }

    public void updateOdometry() {

    }

    public void resetOdometry(Pose2d pose) {

    }

    public void resetGyro() {
        gyro.reset();
    }

    public void resetGyro180() { gyro.reset180(); }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public void configurePID() {

    }

    // For Falcon 500 it is "configureMotionMagic"
    public void configureSmartMotion() {

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

    }

    /**
     *  Gets the position of the left and right NEO motors
     * @return double array of positions [left, right]
     */
    public double[] getPositions(){
       return null;
    }

    /**
     *  Sets the left and right motors to a percent output
     * @param leftPercent
     * @param rightPercent
     */
    public void set(double leftPercent, double rightPercent) {

    }

    // Immediately stops the drivetrain, only use in emergencies
    public void stop() {
        leftMasterCIM.stopMotor();
        rightMasterCIM.stopMotor();
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
        leftMasterCIM.set(val);
    }

    public void driveRight(double val) {
        rightMasterCIM.set(val);
    }

}
