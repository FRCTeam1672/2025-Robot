package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class PositionPIDCommand extends Command{
    
    public SwerveSubsystem mSwerve;
    public final Pose2d goalPose;
    private PPHolonomicDriveController mDriveController = Constants.DrivebaseConstants.PP_CONTROLLER;

    private final Trigger endTrigger;
    private final Trigger endTriggerDebounced;

    private final BooleanPublisher endTriggerLogger = NetworkTableInstance.getDefault().getTable("logging").getBooleanTopic("PositionPIDEndTrigger").publish();

    private PositionPIDCommand(SwerveSubsystem mSwerve, Pose2d goalPose) {
        this.mSwerve = mSwerve;
        this.goalPose = goalPose;

        endTrigger = new Trigger(() -> {
            Pose2d diff = mSwerve.getPose().relativeTo(goalPose);
            var rotation = MathUtil.isNear(
                0.0, 
                diff.getRotation().getRotations(), 
                Constants.DrivebaseConstants.kRotationTolerance.getRotations(), 
                0.0, 
                1
            );
            ChassisSpeeds fieldVelocity = mSwerve.getFieldVelocity();
            double vel = Math.sqrt(fieldVelocity.vxMetersPerSecond * fieldVelocity.vxMetersPerSecond + fieldVelocity.vyMetersPerSecond * fieldVelocity.vyMetersPerSecond);

            boolean position = diff.getTranslation().getNorm() < 0.05;
            boolean speed = vel < 0.05;

            return rotation && position && speed;
        });

        endTriggerDebounced = endTrigger.debounce(0.7);
    }

    public static Command generateCommand(SwerveSubsystem swerve, Pose2d goalPose, Time timeout){
        return new PositionPIDCommand(swerve, goalPose).withTimeout(timeout).finallyDo(() -> {
            swerve.drive(new ChassisSpeeds(0,0,0));
            swerve.lock();
        });
    }

    @Override
    public void initialize() {
        endTriggerLogger.accept(endTrigger.getAsBoolean());
    }

    @Override
    public void execute() {
        PathPlannerTrajectoryState goalState = new PathPlannerTrajectoryState();
        goalState.pose = goalPose;

        endTriggerLogger.accept(endTrigger.getAsBoolean());

        mSwerve.drive(
            mDriveController.calculateRobotRelativeSpeeds(
                mSwerve.getPose(), goalState
            )
        );
    }

    @Override
    public void end(boolean interrupted) {
        endTriggerLogger.accept(endTrigger.getAsBoolean());
    }

    @Override
    public boolean isFinished() {
        return endTriggerDebounced.getAsBoolean();
    }
}