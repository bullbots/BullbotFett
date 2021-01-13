package frc.robot.util;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Shifter {

    private DoubleSolenoid shifterSolenoid;

    public enum Gear {
        LOW,
        HIGH
    }

    private Gear currentGear;

    public Shifter(int lowGearChannel, int highGearChannel) {
        shifterSolenoid = new DoubleSolenoid(lowGearChannel, highGearChannel);
        setGear(Gear.LOW);
    }

    public void shiftHigh(){
        setGear(Gear.HIGH);
    }

    public void shiftLow(){
        setGear(Gear.LOW);
    }
    
    private void setGear(Gear gear){
        currentGear = gear;

        switch (gear) {
            case HIGH:
                shifterSolenoid.set(Value.kReverse);
                break;
            case LOW:
                shifterSolenoid.set(Value.kForward);
                break;
        }
    }

    public Gear getGear(){
        return currentGear;
    }
}