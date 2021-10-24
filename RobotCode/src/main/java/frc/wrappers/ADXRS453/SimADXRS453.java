package frc.wrappers.ADXRS453;

import frc.Constants;

public class SimADXRS453 extends CasseroleADXRS453 {

    double rate;
    double angle;

    @Override
    public void reset() {
        rate = 0;
        angle = 0;
    }

    @Override
    public void calibrate() {
        //nothing to do
        System.out.println("Sim Gyro Calibration Completed!");
    }

    @Override
    public double getRate() {
        return rate;
    }

    @Override
    public double getAngle() {
        return angle;
    }

    @Override
    public boolean isConnected() {
        return true;
    }

    public void simUpdate(double newRate){
        rate = newRate;
        angle += newRate * Constants.Ts;
    }   

    public void simSetAngle(double newAngle){
        rate = 0;
        angle = newAngle;
    }   
    
}
