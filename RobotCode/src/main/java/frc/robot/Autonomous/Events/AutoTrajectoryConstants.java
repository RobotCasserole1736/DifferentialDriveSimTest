package frc.robot.Autonomous.Events;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.util.Units;
import frc.Constants;

public class AutoTrajectoryConstants {

    public static final double MAX_AUTO_DT_SPEED_FPS = Units.feetToMeters(14.0);

    /** Default starting position if autonomous does not provide a different pose. */
    public static final Pose2d DEFAULT_START_POSE = new Pose2d(3.0, 3.0, new Rotation2d());

    public static TrajectoryConfig getConfig(){
        var max_vel_mps = Units.feetToMeters(MAX_AUTO_DT_SPEED_FPS);
        var max_accel_mpss = max_vel_mps / 2.0;
        var cfg = new TrajectoryConfig(max_vel_mps, max_accel_mpss);
        cfg.setKinematics(Constants.kDtKinematics);
        //TODO - voltage/current constraints?
        return cfg;

    }

    
}
