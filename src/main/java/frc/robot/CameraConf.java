package frc.robot;
import java.sql.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonTargetSortMode;
import org.photonvision.PhotonUtils;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


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

    public CameraConf(String camName, double camx, double camy, double camz, double camroll, double campitch,
            double camyaw) {
        cam = new PhotonCamera(camName);
        robotToCam = new Transform3d(new Translation3d(camx, camy, camz), new Rotation3d(
                Units.degreesToRadians(camroll), Units.degreesToRadians(campitch), Units.degreesToRadians(camyaw))); // x+
                                                                                                                     // forward,
                                                                                                                     // y+
                                                                                                                     // left,
                                                                                                                     // z+
                                                                                                                     // up.
                                                                                                                     // reference
                                                                                                                     // https://docs.google.com/document/d/18HxdTfdSlbWWq5aoK3luFQ8dL1g96O6ZJo3THPoD0w0/edit?usp=sharing
        x = camx;
        y = camy;
        z = camz;
        roll = camroll;
        yaw = camyaw;
        pitch = campitch;
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

        double distance = 100;

        if (targetCheck) {
            targets = result.getTargets();
            targetSize = targets.size();
            targetIds = new double[targetSize];
            distances = new double[targetSize];
            if(targetSize <= 0) return false;
            for (int i = 0; i < targetSize; i++) {
                ambiguity += targets.get(i).getPoseAmbiguity();
                distances[i] = PhotonUtils.calculateDistanceToTargetMeters(z,
                        field.getTagPose(targets.get(i).getFiducialId()).get().getZ(), pitch,
                        targets.get(i).getPitch());
                targetIds[i] = (double) targets.get(i).getFiducialId();
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

        return targetCheck;// && distance < 5 || (ambiguity < 0.05 && ambiguity > 0 && distance < 3);
    }

    public double[] getIds(){
        if(camCheck()){
            return targetIds;
        }
        return new double[1];
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
