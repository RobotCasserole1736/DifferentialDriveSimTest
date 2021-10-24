package frc.wrappers.TalonFX;

import frc.robot.Robot;

public class TalonFXFactory {

    public static CasseroleTalonFX makeNewController(int can_id){
        if(Robot.isReal()){
            return new RealTalonFX(can_id);
        } else {
            return new SimTalonFX(can_id);
        }
    }
    
}
