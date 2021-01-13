package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
    private CANSparkMax climb_spark_max = new CANSparkMax(Constants.CLIMB_SPARK_MAX, MotorType.kBrushless);
    private CANPIDController climb_pid_controller = climb_spark_max.getPIDController();
    private CANEncoder climb_encoder = climb_spark_max.getEncoder();

    public Climb() {
        climb_spark_max.setSmartCurrentLimit(60);
        climb_spark_max.setIdleMode(IdleMode.kBrake);
    }

    /** This tells the code to set the climb talon's contol mode and magnitude
     * @param control_mode This is the controm mode of the motor
     * @param magnitude This the magnitude of the motor
     */
    public void set(ControlType control_type, double magnitude) {
        climb_pid_controller.setReference(magnitude, control_type);
    }

    /** This tells the motor what set it's speed to
     * @param speed This is the speed of the motor
     */
    public void set(double speed) {
        climb_spark_max.set(speed);
    }

    public void periodic() {
        SmartDashboard.putNumber("Climb Current", climb_spark_max.getOutputCurrent());
    }
}