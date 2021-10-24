package frc.robot.Autonomous.Modes;

import frc.lib.AutoSequencer.AutoSequencer;
import frc.lib.Autonomous.AutoMode;
import frc.robot.Autonomous.Events.AutoEventJSONTrajectory;

public class DriveFwd extends AutoMode {

    @Override
    public void addStepsToSequencer(AutoSequencer seq) {
        seq.addEvent(new AutoEventJSONTrajectory("driveFwd.wpilib.json")); 
    }
    
}

