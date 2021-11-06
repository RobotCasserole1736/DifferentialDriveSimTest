package frc;

import org.photonvision.SimVisionTarget;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpiutil.math.numbers.N2;

public class Constants {
    //////////////////////////////////////////////////////////////////
    // Drivetrain Physical
    //////////////////////////////////////////////////////////////////

    // Max achievable speed/rotation
    public static final double kMaxSpeed = 3.0; // 3 meters per second.
    public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second.

    public static final double kTrackWidth_in = 25.0;
    public static final double kWheelDiameter_in = 6.0;

    public static final double kDtGearRatio = 1.0/ 12.0;

    // Characterization results
    public static final double DT_kSL = 0.0;  // min voltage to overcome static friction in fwd/rev
    public static final double DT_kVL = 1.98; // volts per (meter per second)
    public static final double DT_kAL = 0.2;  // volts per (meter per second squared)
    public static final double DT_kSA = 0.0;  // min voltage to overcome static friction in rotation
    public static final double DT_kVA = 1.5;  // volts per (radians per second)
    public static final double DT_kAA = 0.3;  // volts per (radians per second squared)

    //////////////////////////////////////////////////////////////////
    // Electrical
    //////////////////////////////////////////////////////////////////
    public static final int kDtLeftLeaderCAN_ID    = 1;
    public static final int kDtLeftFollowerCAN_ID  = 2;
    public static final int kDtRightLeaderCAN_ID   = 3;
    public static final int kDtRightFollowerCAN_ID = 4;
    public static final double kMaxDtAppliedVoltage = 10.0;


    //////////////////////////////////////////////////////////////////
    // PhotonVision Vision Processing
    //////////////////////////////////////////////////////////////////
    // Name configured in the PhotonVision GUI for the camera
    public static final String kCamName = "mainCam";

    // Physical location of the camera on the robot, relative to the center of the
    // robot.
    public static final Transform2d kCameraToRobot = new Transform2d(
                    new Translation2d(-0.25, 0), // in meters
                    new Rotation2d());

    // https://firstfrc.blob.core.windows.net/frc2020/PlayingField/2020FieldDrawing-SeasonSpecific.pdf
    // page 208
    public static final double targetWidth =
            Units.inchesToMeters(41.30) - Units.inchesToMeters(6.70); // meters

    // https://firstfrc.blob.core.windows.net/frc2020/PlayingField/2020FieldDrawing-SeasonSpecific.pdf
    // page 197
    public static final double targetHeight =
            Units.inchesToMeters(98.19) - Units.inchesToMeters(81.19); // meters
    public static final double targetHeightAboveGround = Units.inchesToMeters(81.19); // meters

    // https://firstfrc.blob.core.windows.net/frc2020/PlayingField/LayoutandMarkingDiagram.pdf
    // pages 4 and 5
    public static final double kFarTgtXPos = Units.feetToMeters(54);
    public static final double kFarTgtYPos = Units.feetToMeters(27 / 2) - Units.inchesToMeters(43.75) - Units.inchesToMeters(48.0 / 2.0);
    public static final Pose2d kFarTargetPose = new Pose2d(new Translation2d(kFarTgtXPos, kFarTgtYPos), new Rotation2d(0.0));
    public static final SimVisionTarget kFarTarget = new SimVisionTarget(kFarTargetPose, targetHeightAboveGround, targetWidth, targetHeight);

    //////////////////////////////////////////////////////////////////
    // Nominal Sample Times
    //////////////////////////////////////////////////////////////////
    public static final double Ts = 0.02;

    //////////////////////////////////////////////////////////////////
    // Derived Constants
    //////////////////////////////////////////////////////////////////
    public static final double kWheelRadius_m = Units.inchesToMeters(kWheelDiameter_in/2.0); //Radius of the drivetrain wheels in Meters.
    public static final double kTrackWidth_m = Units.inchesToMeters(kTrackWidth_in);
    public static final DifferentialDriveKinematics kDtKinematics = new DifferentialDriveKinematics(kTrackWidth_m);
    public static final LinearSystem<N2, N2, N2> drivetrainSystem = LinearSystemId.identifyDrivetrainSystem(DT_kVL, DT_kAL, DT_kVA, DT_kAA);
}
