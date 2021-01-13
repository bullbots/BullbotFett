package frc.robot.util;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class SafeTalonSRX extends WPI_TalonSRX{

    private int currentLimit = 40;
    
    public SafeTalonSRX(int deviceNumber){
        super(deviceNumber);

        configFactoryDefault();
        configContinuousCurrentLimit(40);
    }
}