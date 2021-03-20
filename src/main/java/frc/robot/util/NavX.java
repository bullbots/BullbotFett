package frc.robot.util;

import com.kauailabs.navx.frc.AHRS;

public class NavX extends AHRS {

    private double angleDelta = 0;

    public NavX(){
        angleDelta = getAngle();
    }

    @Override
    public double getAngle() {
        var angle =  super.getAngle() - angleDelta;
        angle = ((angle+180) % 360) - 180;
        return angle;
    }

    @Override
    public void reset() {
        angleDelta = getAngle();
    }
    
    public void reset180() {
        angleDelta = getAngle() + 180;
    }
}