package frc.sim.Drivetrain.Vision;

import org.photonvision.SimVisionSystem;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import frc.Constants;

public class Vision {

    // Simulated Vision System.
    // Configure these to match your PhotonVision Camera,
    // pipeline, and LED setup.
    double camDiagFOV = 75.0; // degrees
    double camPitch = 15.0; // degrees
    double camHeightOffGround = 0.85; // meters
    double maxLEDRange = 20; // meters
    int camResolutionWidth = 640; // pixels
    int camResolutionHeight = 480; // pixels
    double minTargetArea = 10; // square pixels

    SimVisionSystem simVision;

    Pose2d curPose;

    public Vision(){

        simVision =
            new SimVisionSystem(
                    Constants.kCamName,
                    camDiagFOV,
                    camPitch,
                    Constants.kCameraToRobot,
                    camHeightOffGround,
                    maxLEDRange,
                    camResolutionWidth,
                    camResolutionHeight,
                    minTargetArea);

        simVision.addSimVisionTarget(Constants.kFarTarget);
        simVision.addSimVisionTarget(Constants.kNearTarget);

    }

    public void update(){
            // Update PhotonVision based on our new robot position.
            simVision.processFrame(curPose);
    }

    public void setCurrentPose(Pose2d poseIn){
        curPose = poseIn;
    }
    
}
