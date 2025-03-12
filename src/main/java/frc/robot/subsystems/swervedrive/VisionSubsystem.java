package frc.robot.subsystems.swervedrive;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.counter.UpDownCounter;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification;
import swervelib.SwerveDrive;

public class VisionSubsystem extends SubsystemBase {
    private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2025ReefscapeWelded
            .loadAprilTagLayoutField();
    private final PhotonCamera frontCam = new PhotonCamera("1672_Camera1");
    private final PhotonCamera backCam = new PhotonCamera("1672_Camera2");

    private final Transform3d frontCamPos = new Transform3d(new Translation3d(Units.inchesToMeters(0),
            Units.inchesToMeters(-11),
            Units.inchesToMeters(25)),
            new Rotation3d(0, Math.toRadians(0), Math.toRadians(-3.3)));

    private final Transform3d backCamPos = new Transform3d(new Translation3d(Units.inchesToMeters(4),
            Units.inchesToMeters(-11),
            Units.inchesToMeters(25)),
            new Rotation3d(0, Math.toRadians(-2), Math.toRadians(179))); // Cam mounted facing forward, half a meter
                                                                        // forward of center, half a meter up from
                                                                        // center.

    private Trigger backCamTrigger = new Trigger(backCam::isConnected);
    private Trigger frontCamTrigger = new Trigger(frontCam::isConnected);


    // Construct PhotonPoseEstimator
    private PhotonPoseEstimator frontPoseEst = new PhotonPoseEstimator(aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, frontCamPos);
    private PhotonPoseEstimator backPoseEst = new PhotonPoseEstimator(aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, backCamPos);

    /** Creates a new VisionSubsystem. */
    public VisionSubsystem() {
        backCamTrigger.onFalse(Commands.runOnce(() -> {
            Elastic.sendNotification(
                new Notification(Notification.NotificationLevel.ERROR, "Back Camera Disconnect", "Rear camera disconnect").withDisplaySeconds(5)
            );
        }));
        frontCamTrigger.onTrue(Commands.runOnce(() -> {
            Elastic.sendNotification(
                new Notification(Notification.NotificationLevel.ERROR, "Front Camera Disconnect", "front camera disconnect").withDisplaySeconds(5)
            );
        }));

    }

    @Override
    public void periodic() {

        SmartDashboard.putBoolean("vision/Rear Camera Connected", backCamTrigger.getAsBoolean());
        SmartDashboard.putBoolean("vision/Front Camera Connected", frontCamTrigger.getAsBoolean());
    }

    public void updatePoseEstimation(SwerveDrive swerve) {
        List<PhotonPipelineResult> frontResults = frontCam.getAllUnreadResults();
        List<PhotonPipelineResult> backResults = backCam.getAllUnreadResults();
        if (!frontResults.isEmpty()) {
            PhotonPipelineResult frontPhotonPipelineResult = frontResults.get(0);
            Optional<EstimatedRobotPose> frontUpdate = frontPoseEst.update(frontPhotonPipelineResult);
            if (frontUpdate.isPresent()) {
                if(frontPhotonPipelineResult.getBestTarget().poseAmbiguity < 0.4) {
                    EstimatedRobotPose estimatedRobotPose = frontUpdate.get();
                swerve.addVisionMeasurement(estimatedRobotPose.estimatedPose.toPose2d(),
                        frontPhotonPipelineResult.getTimestampSeconds());
                }
            }
        }
        if(!backResults.isEmpty()) {
            PhotonPipelineResult backPhotonPipelineResult = backResults.get(0);
            Optional<EstimatedRobotPose> backUpdate = backPoseEst.update(backPhotonPipelineResult);
            if (backUpdate.isPresent()) {
                if(backPhotonPipelineResult.getBestTarget().poseAmbiguity < 0.4) {
                EstimatedRobotPose estimatedRobotPose = backUpdate.get();
                swerve.addVisionMeasurement(estimatedRobotPose.estimatedPose.toPose2d(),
                        backPhotonPipelineResult.getTimestampSeconds());
                }
            }
        }
        
    }
}
