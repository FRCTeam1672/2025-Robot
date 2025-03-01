package frc.robot.subsystems;

import static frc.robot.Constants.ClimbConstants.CLIMB_LIMIT;
import static frc.robot.Constants.ClimbConstants.CLIMB_SPEED;
import static frc.robot.Constants.Tolerances.CLIMB_TOLERANCE;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.Elastic;

public class ClimbSubsystem extends SubsystemBase {
    private SparkMax lClimb = new SparkMax(61, MotorType.kBrushless);
    private SparkMax rClimb = new SparkMax(62, MotorType.kBrushless);

    private SparkMaxConfig config = new SparkMaxConfig();

    private Trigger badClimbTrigger = new Trigger(() -> !isClimbGood());

    public ClimbSubsystem() {
        badClimbTrigger.onTrue(Commands.runOnce(() -> {
            Elastic.sendNotification(new Elastic.Notification(Elastic.Notification.NotificationLevel.ERROR, "Twisted Climb Shaft", "Climb motors have gone out of sync, stopped elevators to not twist climb shaft.", 10000));
        }));

        config.smartCurrentLimit(40);
        config.idleMode(IdleMode.kBrake);
        lClimb.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        config.inverted(true);
        rClimb.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void periodic() {        
        SmartDashboard.putNumber("climb/lHeight", lClimb.getEncoder().getPosition());
        SmartDashboard.putNumber("climb/rHeight", rClimb.getEncoder().getPosition());

        if (badClimbTrigger.getAsBoolean()) {
            lClimb.stopMotor();
            rClimb.stopMotor();
        }
        if (lClimb.getEncoder().getPosition() > CLIMB_LIMIT || rClimb.getEncoder().getPosition() > CLIMB_LIMIT) {
            lClimb.stopMotor();
            rClimb.stopMotor();
        }
    }

    public Command climb() {
        return Commands.run(() -> {
            lClimb.set(CLIMB_SPEED);
            rClimb.set(CLIMB_SPEED);
        }).handleInterrupt(() -> {
            lClimb.stopMotor();
            rClimb.stopMotor();
        });
    }

    public Command unclimb() {
        return Commands.run(() -> {
            lClimb.set(-CLIMB_SPEED);
            rClimb.set(-CLIMB_SPEED);
        }).handleInterrupt(() -> {
            lClimb.stopMotor();
            rClimb.stopMotor();
        });
    }

    public boolean isClimbGood() {
        return Math.abs(lClimb.getEncoder().getPosition() - rClimb.getEncoder().getPosition()) <= CLIMB_TOLERANCE;
    }
}
