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
    Pose2d dtPoseEst = null;

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

    public Pose2d getDtPoseEst(){
        return dtPoseEst;
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
        return (dtPoseEst != null);
    }

    public void update(){

        //Read info from the camera
        var res = cam.getLatestResult();
        var rxTime = Timer.getFPGATimestamp();

        if (res.hasTargets()) {
            var tgt = res.getBestTarget();
            Transform2d camToTargetTrans = tgt.getCameraToTarget();
            dtPoseEst = Constants.kFarTargetPose.transformBy(camToTargetTrans.inverse()).transformBy(Constants.kCameraToRobot);
            imageCaptureTime = rxTime - res.getLatencyMillis();
            tgtAngle = tgt.getYaw();
        } else {
            dtPoseEst = null;
            imageCaptureTime = -1;
            tgtAngle = 0.0;
        }
    }
    
}
