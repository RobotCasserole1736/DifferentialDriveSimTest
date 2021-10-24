package frc.robot.HumanInterfaces;

import edu.wpi.first.wpilibj.XboxController;
import frc.lib.Calibration.Calibration;
import frc.lib.Signal.Annotations.Signal;
import edu.wpi.first.wpilibj.GenericHID.Hand;

public class DriverController {

    /* Singleton infratructure*/
    private static DriverController inst = null;
    public static synchronized DriverController getInstance() {
        if (inst == null)
            inst = new DriverController();
        return inst;
    }

    Calibration deadBandCal;

    XboxController xbCtrl;

    @Signal
    double xSpeedCmd;
    @Signal
    double zRotateCmd;

    private DriverController(){
        xbCtrl = new XboxController(0);
        deadBandCal = new Calibration("Driver Controller Joystick Deadzone", "cmd", 0.10, 0.0, 1.0);
    }

    public double getXSpeedCmd(){
        return xSpeedCmd;
    }

    public double getZRotateCmd(){
        return zRotateCmd;
    }

    public void update(){
        xSpeedCmd = applyDeadband(-1.0*xbCtrl.getY(Hand.kLeft));
        zRotateCmd = applyDeadband(-1.0*xbCtrl.getX(Hand.kRight));
    }

    private double applyDeadband(double value) {
        double deadband = deadBandCal.get();
        if (Math.abs(value) > deadband) {
          if (value > 0.0) {
            return (value - deadband) / (1.0 - deadband);
          } else {
            return (value + deadband) / (1.0 - deadband);
          }
        } else {
          return 0.0;
        }
      }
    
}
