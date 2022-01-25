package frc.robot.util;


public class SafeSparkMax extends CANSparkMax {

    private int stallLimit = 40;
    private int freeLimit = 40;

    public SafeSparkMax(int deviceNumber, MotorType motorType) {

    }

}
