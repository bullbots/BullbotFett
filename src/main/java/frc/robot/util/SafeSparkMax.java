package frc.robot.util;

import com.revrobotics.CANSparkMax;

public class SafeSparkMax extends CANSparkMax {

    private int stallLimit = 40;
    private int freeLimit = 40;

    public SafeSparkMax(int deviceNumber, MotorType motorType) {
        super(deviceNumber, motorType);

        clearFaults();
        setSmartCurrentLimit(stallLimit, freeLimit);
        burnFlash();

        restoreFactoryDefaults();
    }
}