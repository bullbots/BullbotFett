package frc.robot.util;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class SafeVictorSPX extends WPI_VictorSPX {

    public SafeVictorSPX(int deviceNumber) {
        super(deviceNumber);
    }
}