package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import static frc.robot.Constants.ReefPose.*;

public record ReefAlignment(ReefOrientation orientation, ReefSide side) {

    public Pose2d getAlignmentPose() {
        return getAlignmentPose(FRONT_BACK_OFFSET);
    }
    public Pose2d getInitalPose() {
        return getAlignmentPose(INITIAL_ALIGNMENT_OFFSET);
    }

    private Pose2d getAlignmentPose(Translation2d forwardsBackwardsOffset) {
        Translation2d reefPose = BLUE_REEF_CENTER.minus(forwardsBackwardsOffset).plus(centerOffset.times(side == ReefSide.LEFT ? 1 : -1)).minus(leftRightOffset);
        reefPose = reefPose.rotateAround(BLUE_REEF_CENTER, Rotation2d.fromDegrees(orientation.getReefRotation()));
        return flipPose(new Pose2d(reefPose, Rotation2d.fromDegrees(orientation.getRobotRotation())));
    }

    public Pose2d getCenterPose() {
        Translation2d reefPose = BLUE_REEF_CENTER.minus(new Translation2d(2, 0)).plus(centerOffset.times(side == ReefSide.LEFT ? 1 : -1));
        reefPose = reefPose.rotateAround(BLUE_REEF_CENTER, Rotation2d.fromDegrees(orientation.getReefRotation()));
        return flipPose(new Pose2d(reefPose, Rotation2d.fromDegrees(orientation.getRobotRotation())));
    }

    private Pose2d flipPose(Pose2d pose) {
        if(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue){
            System.out.println("Not flipping");
            return pose;
        }
        System.out.println("Flipping");
        Translation2d center = BLUE_REEF_CENTER.interpolate(RED_REEF_CENTER, 0.5);
        Translation2d poseTranslation = pose.getTranslation();
        poseTranslation = poseTranslation.rotateAround(center, Rotation2d.k180deg);
        return new Pose2d(poseTranslation, pose.getRotation().rotateBy(Rotation2d.k180deg));
    }

}
