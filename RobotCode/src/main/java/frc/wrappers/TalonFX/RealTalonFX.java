package frc.wrappers.TalonFX;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.util.Units;

public class RealTalonFX extends CasseroleTalonFX {

    final int TIMEOUT_MS = 30;

    final double MAX_VOLTAGE = 14.0;


    public RealTalonFX(int can_id){
        _talon = new WPI_TalonFX(can_id);
        _talon.configFactoryDefault();
        _talon.configNeutralDeadband(0.001);
        _talon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
                                               0, 
                                               TIMEOUT_MS);
        _talon.configNominalOutputForward(0, TIMEOUT_MS);
        _talon.configNominalOutputReverse(0, TIMEOUT_MS);
        _talon.configPeakOutputForward(1,  TIMEOUT_MS);
        _talon.configPeakOutputReverse(-1, TIMEOUT_MS);
        _talon.enableVoltageCompensation(true);
        _talon.configVoltageCompSaturation(MAX_VOLTAGE, TIMEOUT_MS);
    }


    @Override
    public void setInverted(boolean invert) {
        _talon.setInverted(invert);
    }


    @Override
    public void setClosedLoopGains(double p, double i, double d, double f) {
        _talon.config_kP(0, p, TIMEOUT_MS);
        _talon.config_kI(0, i, TIMEOUT_MS);
        _talon.config_kD(0, d, TIMEOUT_MS);
        _talon.config_kF(0, f, TIMEOUT_MS);        
    }


    @Override
    public void setVelocityCmd(double velocity_radpersec) {
        var velCmdRPM = Units.radiansPerSecondToRotationsPerMinute(velocity_radpersec);
        _talon.set(TalonFXControlMode.Velocity, RPMtoCTRENativeUnits(velCmdRPM));
    }


    @Override
    public void setVoltageCmd(double cmd_v) {
        var pctCmd = cmd_v/MAX_VOLTAGE;

        if(pctCmd > 1.0){
            pctCmd = 1.0;
        }

        if(pctCmd < -1.0){
            pctCmd = -1.0;
        }

        _talon.set(TalonFXControlMode.PercentOutput, pctCmd);
    }


    @Override
    public double getCurrent_A() {
        return _talon.getStatorCurrent();
    }


    @Override
    public double getVelocity_radpersec() {
        var velRPM = CTRENativeUnitstoRPM(_talon.getSelectedSensorVelocity(0));
        return Units.rotationsPerMinuteToRadiansPerSecond(velRPM);
    }


    @Override
    public void follow(CasseroleTalonFX leader) {
        _talon.follow(leader._talon);
    }


    @Override
    public double getPosition_rad() {
        var posRev = CTRENativeUnitstoRev(_talon.getSelectedSensorPosition(0));
        return posRev * 2 * Math.PI;
    }
    
}
