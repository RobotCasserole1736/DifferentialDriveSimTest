package frc.wrappers.TalonFX;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.wrappers.SimCANDevice;

public abstract class CasseroleTalonFX extends SimCANDevice {

    WPI_TalonFX _talon;

    ArrayList<CasseroleTalonFX> simFollowers = new ArrayList<CasseroleTalonFX>();

    //Falcon-500 specific internal encoder conversion factor
    public final double NATIVE_UNITS_PER_REV = 2048.0;
    
    public abstract void setInverted(boolean invert);
    public abstract void setClosedLoopGains(double p, double i, double d);
    public abstract void setClosedLoopCmd(double velocityCmd_radpersec, double arbFF_fracSupply);
    public abstract void setVoltageCmd(double cmd_v);
    public abstract double getCurrent_A();
    public abstract double getVelocity_radpersec();
    public abstract double getPosition_rad();
    public abstract void follow(CasseroleTalonFX leader);

    
    double RPMtoCTRENativeUnits(double in_rpm){
        return in_rpm * NATIVE_UNITS_PER_REV / 600.0;
    }

    double CTRENativeUnitstoRPM(double in_native){
        return in_native / NATIVE_UNITS_PER_REV * 600.0;
    }

    double RevtoCTRENativeUnits(double in_rev){
        return in_rev * NATIVE_UNITS_PER_REV ;
    }

    double CTRENativeUnitstoRev(double in_native){
        return in_native / NATIVE_UNITS_PER_REV;
    }

}
