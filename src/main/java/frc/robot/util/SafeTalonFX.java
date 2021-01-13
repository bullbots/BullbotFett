package frc.robot.util;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class SafeTalonFX extends WPI_TalonFX {

    private int currentLimit = 40;
    private int currentThreshold = 0;
    private double currentThresholdTime = 0; // In Milliseconds
    private boolean usePID;
    private int maxSpeed = 21000;

    public SafeTalonFX(int deviceNumber){
        this(deviceNumber, false);
    }

    public SafeTalonFX(int deviceNumber, boolean usesPID) {
        super(deviceNumber);

        this.usePID = usesPID;

        configFactoryDefault();

        StatorCurrentLimitConfiguration config = new StatorCurrentLimitConfiguration(
            true,
            currentLimit,
            currentThreshold,
            currentThresholdTime
        );

        configStatorCurrentLimit(config);
    }

    /**
     * Moves the motor at a percent of its max speed
     * @param percentOutput A double from -1 to 1
     * 
     */
    @Override
    public void set(double percentOutput) {
        if (usePID) {
            super.set(ControlMode.Velocity, percentOutput * maxSpeed);
        }else {
            super.set(percentOutput);
        }
    }
}