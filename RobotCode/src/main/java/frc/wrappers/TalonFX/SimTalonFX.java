package frc.wrappers.TalonFX;

import edu.wpi.first.wpilibj.util.Units;
import frc.Constants;
import frc.lib.Signal.Annotations.Signal;
import frc.wrappers.SimCANDeviceBank;

/**
 * CTRE doesn't currently support simulating the internal functionality of a TalonFX.
 * We provide a wrapper and interface abstraction here to enable simulation through
 * a transparent wrapper layer in robot code.
 */
public class SimTalonFX extends CasseroleTalonFX {

    private boolean isInverted; //TODO use me?
    private double kP;
    private double kI;
    private double kD;

    @Signal
    private double curWindingVoltage;
    @Signal
    private double curCurrent;
    @Signal
    private double curVel_radpersec;
    @Signal
    private double curSupplyVoltage = 12.0;
    @Signal
    private double curPos_rad;

    public SimTalonFX(int can_id){
        this.can_id = can_id;
        SimCANDeviceBank.add(this);
    }


    @Override
    public void setInverted(boolean invert) {
        isInverted = invert;
    }


    @Override
    public void setClosedLoopGains(double p, double i, double d) {
        kP = p;
        kI = i;
        kD = d;
    }


    @Override
    public void setClosedLoopCmd(double velocityCmd_radpersec, double arbFF_fracSupply) {
        setVoltageCmd(ctrePIDSim(velocityCmd_radpersec, arbFF_fracSupply));
    }


    @Override
    public void setVoltageCmd(double cmd_v) {
        curWindingVoltage = limitVoltage(cmd_v);
        for(CasseroleTalonFX follower : simFollowers){
            follower.setVoltageCmd(curWindingVoltage);
        }
    }


    @Override
    public double getCurrent_A() {
        return curCurrent;
    }

    @Override
    public double getVelocity_radpersec() {
        return curVel_radpersec;
    }

    @Override
    public double getPosition_rad() {
        return curPos_rad;
    }

    public void sim_setActualVelocity(double velocity_radpersec){
        curVel_radpersec = velocity_radpersec;
    }

    public void sim_setActualPosition(double pos_rad){
        curPos_rad = pos_rad;
    }

    public double sim_getWindingVoltage(){
        return curWindingVoltage;
    }

    public void sim_setSupplyVoltage(double supply_V){
        curSupplyVoltage = supply_V;
    }

    public void sim_setCurrent(double cur_A){
        curCurrent = cur_A;
    }

    private double limitVoltage(double in){
        if(in > curSupplyVoltage){
            return curSupplyVoltage;
        } else if (in < -curSupplyVoltage){
            return -curSupplyVoltage;
        } else {
            return in;
        }
    }


    double velErrNative_accum;
    double velErrNative_prev;
    /**
     * A rough guess at the behavior of the CTRE controller from 
     * https://docs.ctre-phoenix.com/en/stable/ch16_ClosedLoop.html#closed-loop-configurations
     */
    private double ctrePIDSim(double vel_cmd, double arb_ff_frac){
        var velError_RPM = Units.radiansPerSecondToRotationsPerMinute(curVel_radpersec - vel_cmd);
        var velErrorNative = RPMtoCTRENativeUnits(velError_RPM);
        
        velErrNative_accum += velErrorNative;
        
        var velErrNative_delta = (velErrorNative - velErrNative_prev)/Constants.Ts;

        var pTerm = velErrorNative * kP;
        var dTerm = velErrNative_delta * kD;
        var iTerm = velErrNative_accum * kI;
        var fTerm = arb_ff_frac;

        velErrNative_prev = velErrorNative;

        return limitVoltage(((pTerm + dTerm + iTerm) / 1023.0 + fTerm) * curSupplyVoltage);
    }


    @Override
    public void follow(CasseroleTalonFX leader) {
        leader.simFollowers.add(this);
    }

    
}
