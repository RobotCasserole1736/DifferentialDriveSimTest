package frc.robot;

import javax.annotation.Signed;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import frc.lib.Signal.Annotations.Signal;

public class BatteryMonitor {

    /* Singleton infratructure*/
    private static BatteryMonitor inst = null;
    public static synchronized BatteryMonitor getInstance() {
        if (inst == null)
            inst = new BatteryMonitor();
        return inst;
    }

    PowerDistributionPanel pdp;

    @Signal
    double batteryVoltage;

    @Signal
    double batteryCurrent;

    private BatteryMonitor(){
        pdp = new PowerDistributionPanel(0);
    }

    public void update(){
        batteryVoltage = pdp.getVoltage();
        batteryCurrent = pdp.getTotalCurrent();
    }

    public double getCurBatteryVoltage(){
        return batteryVoltage;
    }
    
}
