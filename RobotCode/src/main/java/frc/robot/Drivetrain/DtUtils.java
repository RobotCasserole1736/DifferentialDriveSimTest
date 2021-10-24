package frc.robot.Drivetrain;

import frc.Constants;

public class DtUtils {

    public static double MotorRotationRad_to_DtLinM(double rot_in){ return rot_in * (Constants.kWheelRadius_m * Constants.kDtGearRatio); }
    public static double DtLinM_to_MotorRotationRad(double lin_in){ return lin_in / (Constants.kWheelRadius_m * Constants.kDtGearRatio); }
    
}
