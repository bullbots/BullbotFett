package frc.robot.util;

import com.kauailabs.navx.frc.AHRS;

public class NavX extends AHRS {

    private double angleDelta = 0;

    public NavX(){
        super();

        angleDelta = super.getAngle();
    }

    @Override
    public double getAngle(){
        return super.getAngle() - angleDelta;
    }

    @Override
    public void reset() {
        angleDelta = super.getAngle();
    }
}