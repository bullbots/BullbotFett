package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;
import frc.robot.Robot;

import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

import frc.robot.util.SafeSparkMax;

public class Shooter extends SubsystemBase {

    private SafeSparkMax top_shooter;
    private SafeSparkMax bottom_shooter;

    private CANEncoder top_encoder;
    private CANEncoder bottom_encoder;

    private CANPIDController top_pid_controller;
    private CANPIDController bottom_pid_controller;

    public final Servo ballReleaseServo = new Servo(Constants.RELEASE_SERVO_PORT);

    private final DoubleSolenoid angle_solenoid = new DoubleSolenoid(Constants.HIGH_ANGLE_CHANNEL, Constants.LOW_ANGLE_CHANNEL);

    private NetworkTableEntry topVelocity;
    private NetworkTableEntry bottomVelocity;

    List<Integer> velocityRange = IntStream.rangeClosed(-100, 100).boxed().collect(Collectors.toList());
    private int curTopVelIndex = 0;
    private int curBottomVelIndex = velocityRange.size() / 2;

    private enum MotorPlacement {
        BOTTOM, TOP
    }

    public Shooter(){
        // top_shooter = new SafeSparkMax(Constants.TOP_SHOOTER_PORT, MotorType.kBrushless);
        // bottom_shooter = new SafeSparkMax(Constants.BOTTOM_SHOOTER_PORT, MotorType.kBrushless);
      
        // Always reset to factory defaults, just in case.
        top_shooter.restoreFactoryDefaults();
        bottom_shooter.restoreFactoryDefaults();
      
        top_shooter.clearFaults();
        bottom_shooter.clearFaults();

        top_encoder = top_shooter.getEncoder();
        bottom_encoder = bottom_shooter.getEncoder();

        top_pid_controller = top_shooter.getPIDController();
        bottom_pid_controller = bottom_shooter.getPIDController();

        angle_solenoid.set(Value.kForward);

        configurePID();
        configureShuffleBoard();
    }

    private void configurePID() {
        top_pid_controller.setFF(Constants.SHOOTER_FF);
        top_pid_controller.setP(Constants.SHOOTER_P);
        top_pid_controller.setI(Constants.SHOOTER_I);
        top_pid_controller.setD(Constants.SHOOTER_D);

        bottom_pid_controller.setFF(Constants.SHOOTER_FF);
        bottom_pid_controller.setP(Constants.SHOOTER_P);
        bottom_pid_controller.setI(Constants.SHOOTER_I);
        bottom_pid_controller.setD(Constants.SHOOTER_D);
    }

    public void raiseSolenoid() {
        angle_solenoid.set(Value.kForward);
    }

    public void lowerSolenoid() {
        angle_solenoid.set(Value.kReverse);
    }

    public void stop() {
        top_shooter.stopMotor();
        bottom_shooter.stopMotor();
    }

    private void configureShuffleBoard() {
        topVelocity = Shuffleboard.getTab("Diagnostics")
                .add("Top Encoder Velocity", 0)
                .withSize(2, 2)
                .withPosition(0, 4)
                .withWidget(BuiltInWidgets.kGraph)
                .getEntry();
        bottomVelocity = Shuffleboard.getTab("Diagnostics")
                .add("Bottom Encoder Velocity", 0)
                .withSize(2, 2)
                .withPosition(0, 4)
                .withWidget(BuiltInWidgets.kGraph)
                .getEntry();

        // Shuffleboard.getTab("Diagnostics")
        //         .add("Bottom Encoder Velocity", 0)
        //         .withSize(2, 2)
        //         .withPosition(2, 4)
        //         .withWidget(BuiltInWidgets.kGraph)
        //         .getEntry();
    }

    /**This gets the velocity of the motors
     * @param motorPlacement This is where the motor is placed either at the top or bottem
     * @return curVal This retuns the curent velocity of the shooter motors
     */
    public double getVelocity(MotorPlacement motorPlacement) {
        double curVal = 0;
        if (Robot.isReal()) {
            switch (motorPlacement) {
                case TOP:
                    curVal = top_encoder.getVelocity();
                    break;
                case BOTTOM:
                    curVal = bottom_encoder.getVelocity();
            }
        } else {
            int curIndex = 0;

            switch (motorPlacement) {
                case TOP:
                    curIndex = curTopVelIndex;
                    break;
                case BOTTOM:
                    curIndex = curBottomVelIndex;
            }

            curVal = velocityRange.get(curIndex);

            if (curIndex >= velocityRange.size() - 1) {
                switch (motorPlacement) {
                    case TOP:
                        curTopVelIndex = 0;
                        break;
                    case BOTTOM:
                        curBottomVelIndex = 0;
                        break;
                }
            } else {
                switch (motorPlacement) {
                    case TOP:
                        curTopVelIndex++;
                        break;
                    case BOTTOM:
                        curBottomVelIndex++;
                        break;
                }
            }
        }
        return curVal;
    }

    public double[] getVelocities() {
        double top_vel = top_encoder.getVelocity();
        double bottom_vel = bottom_encoder.getVelocity();

        return new double[] {top_vel, bottom_vel};
    }

    /** This sets the velocity of the top abd bottem shooter motors
     * @param top_vel This is the velocity of the top motor
     * @param bottom_vel This is the velocity of the bottem mtor
     */
    public void set(double top_vel, double bottom_vel){
        top_pid_controller.setReference(top_vel, ControlType.kVelocity);
        bottom_pid_controller.setReference(bottom_vel, ControlType.kVelocity);
    }

    public void periodic() {
        // double kF = top_pid_controller.getFF();
        // double kP = top_pid_controller.getP();
        // double kI = top_pid_controller.getI();
        // double kD = top_pid_controller.getD();

        // double newF = SmartDashboard.getNumber("Shooter F", kF);
        // double newP = SmartDashboard.getNumber("Shooter P", kP);
        // double newI = SmartDashboard.getNumber("Shooter I", kI);
        // double newD = SmartDashboard.getNumber("Shooter D", kD);

        // top_pid_controller.setFF(newF);
        // top_pid_controller.setP(newP);
        // top_pid_controller.setI(newI);
        // top_pid_controller.setD(newD);
        SmartDashboard.putNumber("ServoValue", ballReleaseServo.getAngle());

        topVelocity.setDouble(getVelocity(MotorPlacement.TOP));
        bottomVelocity.setDouble(getVelocity(MotorPlacement.BOTTOM));
    }

    public boolean angle45() {
        return angle_solenoid.get().equals(Value.kForward);
    }
}