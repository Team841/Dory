package com.team841.dory.drive;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.team254.vision.VisionFieldPoseEstimate;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.littletonrobotics.junction.AutoLog;

import java.util.function.Supplier;

/**
 * IO interface for drivetrain following Advantage Kit format, we only log SwerveDrivetrain.SwerveDriveState class
 */
public interface DriveIO {

    @AutoLog
    class DriveIOInputs extends SwerveDrivetrain.SwerveDriveState {
        public double gyroAngle = 0.0;

        DriveIOInputs() {
            this.Pose = Pose2d.kZero;
        }

        public void fromSwerveDriveState(SwerveDrivetrain.SwerveDriveState inputState) {
            this.Pose = inputState.Pose;
            this.SuccessfulDaqs = inputState.SuccessfulDaqs;
            this.FailedDaqs = inputState.FailedDaqs;
            this.ModuleStates = inputState.ModuleStates;
            this.ModuleTargets = inputState.ModuleTargets;
            this.Speeds = inputState.Speeds;
            this.OdometryPeriod = inputState.OdometryPeriod;
        }
    }

    void updateInputs(DriveIOInputs inputs);

    void logModules(SwerveDrivetrain.SwerveDriveState state);

    void seedFieldCentric();

    void seedFieldRelative(Pose2d pose);

    Pose2d getPose();

    void setControl(SwerveRequest request);

    Command applyRequest(Supplier<SwerveRequest> requestSupplier, Subsystem subsystemRequired);

    void addVisionMeasurement(VisionFieldPoseEstimate visionFieldPoseEstimate);

    void setOperatorPerspectiveForward(Rotation2d fieldDirection);

    void pointModulesAtAngle();
}
