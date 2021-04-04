package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.Drivetrain_Commands.ShiftLow;

public class Shifter extends SubsystemBase {

    private DoubleSolenoid shifterSolenoid;

    public enum Gear {
        LOW,
        HIGH
    }

    private Gear currentGear;

    private int lowGearChannel = Constants.LOW_GEAR_CHANNEL;
    private int highGearChannel = Constants.HIGH_GEAR_CHANNEL;

    public Shifter() {
        shifterSolenoid = new DoubleSolenoid(lowGearChannel,     highGearChannel);
        setGear(Gear.LOW);

        setDefaultCommand(new ShiftLow(this));
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