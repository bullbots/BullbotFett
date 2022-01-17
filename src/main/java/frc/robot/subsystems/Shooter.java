package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import frc.robot.Robot;

import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

import frc.robot.util.SafeTalonFX;

public class Shooter extends SubsystemBase {

    private SafeTalonFX top_shooter;
    private SafeTalonFX bottom_shooter;

    // public final Servo ballReleaseServo = new Servo(Constants.RELEASE_SERVO_PORT);

    private NetworkTableEntry topVelocity;
    private NetworkTableEntry bottomVelocity;

    List<Integer> velocityRange = IntStream.rangeClosed(-100, 100).boxed().collect(Collectors.toList());
    private int curTopVelIndex = 0;
    private int curBottomVelIndex = velocityRange.size() / 2;

    private enum MotorPlacement {
        BOTTOM, TOP
    }

    public Shooter(){
        top_shooter = new SafeTalonFX(Constants.TOP_SHOOTER_PORT, true);
        bottom_shooter = new SafeTalonFX(Constants.BOTTOM_SHOOTER_PORT, true);

        configurePID();

        top_shooter.setNeutralMode(NeutralMode.Coast);
        bottom_shooter.setNeutralMode(NeutralMode.Coast);

        // setDefaultCommand(new RunCommand(() -> ballReleaseServo.set(1), this));

        var inst = NetworkTableInstance.getDefault();
        // topVelocity = inst.getEntry("Shooter Top Velocity");
        // bottomVelocity = inst.getEntry("Shooter Bottom Velocity");
    }

    private void configurePID() {
        top_shooter.config_kF(0, Constants.SHOOTER_FF);
        top_shooter.config_kP(0, Constants.SHOOTER_P);
        top_shooter.config_kI(0, Constants.SHOOTER_I);
        top_shooter.config_kD(0, Constants.SHOOTER_D);

        bottom_shooter.config_kF(0, Constants.SHOOTER_FF);
        bottom_shooter.config_kP(0, Constants.SHOOTER_P);
        bottom_shooter.config_kI(0, Constants.SHOOTER_I);
        bottom_shooter.config_kD(0, Constants.SHOOTER_D);
    }

    public void stop() {
        top_shooter.stopMotor();
        bottom_shooter.stopMotor();
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
                    curVal = top_shooter.getSelectedSensorVelocity();
                    break;
                case BOTTOM:
                    curVal = bottom_shooter.getSelectedSensorVelocity();
            }
        } else {
            int curIndex = 0;

            switch (motorPlacement) {
                case TOP:
                    curIndex = curTopVelIndex;
                    break;
                case BOTTOM:
                    curIndex = curBottomVelIndex;
                    break;
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
        double top_vel = top_shooter.getSelectedSensorVelocity();
        double bottom_vel = bottom_shooter.getSelectedSensorVelocity();

        return new double[] {top_vel, bottom_vel};
    }

    /** This sets the velocity of the top abd bottem shooter motors
     * @param top_vel This is the velocity of the top motor
     * @param bottom_vel This is the velocity of the bottem mtor
     */
    public void set(double top_vel, double bottom_vel){
        top_shooter.set(top_vel);
        bottom_shooter.set(bottom_vel);
        System.out.println(String.format("Top Velocity %.02f Bottom Velocity %.02f", top_vel, bottom_vel));
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
        // SmartDashboard.putNumber("ServoValue", ballReleaseServo.getAngle());

        // topVelocity.setDouble(getVelocity(MotorPlacement.TOP));
        // bottomVelocity.setDouble(getVelocity(MotorPlacement.BOTTOM));
    }
}