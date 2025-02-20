package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.LimelightConstants;
import frc.robot.LimelightHelpers.PoseEstimate;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class LimelightSubsystem extends SubsystemBase {
    public static String LimelightID = LimelightConstants.LimelightCoral;
    private boolean detecting = false;
    
    public String getLimelightName() {
        return LimelightID;
    }

    public double getID() {
        return LimelightHelpers.getFiducialID(LimelightID);
    }

    public PoseEstimate getMeasurement(double angle){
        LimelightHelpers.SetRobotOrientation(LimelightID, angle, 0.0, 0.0, 0.0, 0.0, 0.0);
        LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LimelightID);

        return limelightMeasurement;
    }
    
    public Pose2d getRobotPose(){
        LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(LimelightID);
        return new Pose2d(limelightMeasurement.pose.getX(), limelightMeasurement.pose.getY(), limelightMeasurement.pose.getRotation());
    }

    public double getPoseX(){
        return getRobotPose().getX();
    }

    public double getPoseY(){
        return getRobotPose().getY();
    }
    
    public Rotation2d getPoseRotation2d(){
        return getRobotPose().getRotation();
    }
    
    public double getPipeline(){
        return LimelightHelpers.getCurrentPipelineIndex(LimelightID);
    }

    public void setPipeline(int pipeline){
        LimelightHelpers.setPipelineIndex(LimelightID, pipeline);
    }

    public double getTA(){
        return LimelightHelpers.getTA(LimelightID);
    }

    public double getTX(){
        return LimelightHelpers.getTX(LimelightID);
    }

    public double getTY(){
        return LimelightHelpers.getTY(LimelightID);
    }

    public boolean detecting(){
        return detecting;
    }

    @Override
    public void periodic() {
        //if(LimelightHelpers.getFiducialID(LimelightConstants.LimelightReef) > 0 && LimelightHelpers.getTA(LimelightConstants.LimelightReef) > 0.6){
        //    LimelightID = LimelightConstants.LimelightReef;
        //} else if(LimelightHelpers.getFiducialID(LimelightConstants.LimelightCoral) > 0 && LimelightHelpers.getTA(LimelightConstants.LimelightCoral) > 0.6){
        //        LimelightID = LimelightConstants.LimelightCoral;
        //}

        if(getID() > 0){
            if(LimelightID == LimelightConstants.LimelightReef){
                detecting = true;
            }

            if(LimelightID == LimelightConstants.LimelightCoral && getTA() > 1.5){
                detecting = true;
            }

        } else {
            detecting = false;
        }

    }

}
