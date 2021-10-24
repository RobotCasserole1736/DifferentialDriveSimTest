package frc.sim;

import frc.sim.Drivetrain.DrivetrainPlant;

public class Plant {

    DrivetrainPlant m_dt;

    public Plant(){
        m_dt = new DrivetrainPlant();
    }

    public void update() {
        m_dt.update();
    }
    
}
