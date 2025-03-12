package frc.robot;
import java.util.List;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;


public class CameraConf {
    public PhotonCamera cam;
    public PhotonPipelineResult result;
    public AprilTagFieldLayout field;
    public Transform3d robotToCam;

    public PhotonPoseEstimator photonPoseEstimator;

    public int targetSize;
    public double targetIds[];
    public double distances[];
    public double ambiguity = 0;
    public double x = 0;
    public double y = 0;
    public double z = 0;
    public double roll = 0;
    public double yaw = 0;
    public double pitch = 0;
    public double distance = 0;
    public CameraConf(String camName, double camx, double camy, double camz, double camroll, double campitch,
            double camyaw) {
        x = camx;
        y = camy;
        z = camz;
        roll = Units.degreesToRadians(camroll);
        yaw = Units.degreesToRadians(camyaw);
        pitch = Units.degreesToRadians(campitch);

        cam = new PhotonCamera(camName);
        robotToCam = new Transform3d(new Translation3d(x, y, z), new Rotation3d(roll, pitch, yaw));
        // x+ = forward, y+ = left, z+ = up. reference https://docs.google.com/document/d/18HxdTfdSlbWWq5aoK3luFQ8dL1g96O6ZJo3THPoD0w0/edit?usp=sharing
        field = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
        photonPoseEstimator = new PhotonPoseEstimator(field, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);
        photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        result = new PhotonPipelineResult();

    }

public void refresh(){
        result = cam.getLatestResult();
    }


    public Boolean camCheck() {
        List<PhotonTrackedTarget> targets;
        boolean targetCheck = result.hasTargets();


        if (targetCheck) {
            targets = result.getTargets();
            targetSize = targets.size();
            targetIds = new double[targetSize];
            distances = new double[targetSize];
            PhotonTrackedTarget currentTarget;
            if(targetSize <= 0) return false;
            for (int i = 0; i < targetSize; i++) {
                if(targets.get(i) != null){
                    currentTarget = targets.get(i);
                }else{
                    continue;
                }
                ambiguity += currentTarget.getPoseAmbiguity();
                distances[i] = currentTarget.getBestCameraToTarget().getTranslation().getDistance(new Translation3d(0,0,0));
                targetIds[i] = (double) currentTarget.getFiducialId();
            }
            ambiguity /= (double) targetSize;
            double minDist = 1000;
            for (int i = 0; i < distances.length; i++) {
                if (distances[i] < minDist) {
                    minDist = distances[i];
                    distance = distances[i];
                }
            }
        }
        return targetCheck && (distance < 3 || (ambiguity < 0.05 && ambiguity > 0 && distance < 5));
    }

    /*public String getIds(){
        String out = "-1";
        if(camCheck()){
            out = "";
            for(int i = 0; i < targetSize; i++){
                out = out  + targetIds[i] + ", ";
            }
        }
        System.out.println(out);
        return out;
    }*/
    public double[] getIds(){
        if(camCheck()){
            return targetIds;
        }
        return new double[]{-1};
    }
    public double getDistance(){
        return distance;
    }

    public Pose2d getEstimatedGlobalPose(Pose2d previousPosition) {

        if (!camCheck()) {

            return previousPosition;
        }

        photonPoseEstimator.setReferencePose(previousPosition);
        try{

            EstimatedRobotPose e = photonPoseEstimator.update(result).get();
            return e.estimatedPose.toPose2d();
    }catch(Exception e){
        //System.out.println(" crahsing dumbass");
    }


       // System.out.println("WE MADE IT");
        return previousPosition;
    }
}
