package frc.robot;

import org.photonvision.PhotonCamera;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import frc.Constants;

public class Vision {

    private PhotonCamera cam;
    double imageCaptureTime = -1;
    Transform2d camToTargetTrans = null;

    double tgtAngle = 0.0;

    NetworkTableEntry rawBytesEntry;


    /* Singleton infratructure*/
    private static Vision inst = null;
    public static synchronized Vision getInstance() {
        if (inst == null)
            inst = new Vision();
        return inst;
    }
    
    private Vision(){
        cam = new PhotonCamera(Constants.kCamName);
    }

    public Pose2d getDtPoseEst(boolean isFar){
        Pose2d tgtPose = isFar?Constants.kFarTargetPose:Constants.kNearTargetPose;
        return tgtPose.transformBy(camToTargetTrans.inverse()).transformBy(Constants.kCameraToRobot);
    }

    public double getCaptureTime(){
        return imageCaptureTime;
    }

    public double getTgtAngle(){
        return tgtAngle;
    }

    public boolean getCamOnline(){
        return true; //todo - detect photon camera not updating raw bytes entry?
    }

    public boolean targetVisble(){
        return (camToTargetTrans != null);
    }

    public void update(){

        //Read info from the camera
        var res = cam.getLatestResult();
        var rxTime = Timer.getFPGATimestamp();

        if (res.hasTargets()) {
            var tgt = res.getBestTarget();
            camToTargetTrans = tgt.getCameraToTarget();
            imageCaptureTime = rxTime - res.getLatencyMillis();
            tgtAngle = tgt.getYaw();
        } else {
            camToTargetTrans = null;
            imageCaptureTime = -1;
            tgtAngle = 0.0;
        }
    }
    
}
