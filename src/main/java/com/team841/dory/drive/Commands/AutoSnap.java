package com.team841.dory.drive.Commands;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.team841.dory.drive.Drivetrain;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

import org.littletonrobotics.junction.Logger;

public class AutoSnap extends Command {

    Drivetrain drivetrain;
    ProfiledPIDController vx, vy;
    Pose2d target;

    private final SwerveRequest.FieldCentricFacingAngle driveHeading = new SwerveRequest.FieldCentricFacingAngle()
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

    public AutoSnap(Drivetrain drivetrain) {
        this.vx = drivetrain.vxController;
        this.vy = drivetrain.vyController;
        this.drivetrain = drivetrain;

        addRequirements(this.drivetrain);
        setName("AutoSnap");
    }

    @Override
    public void initialize() {
        ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                this.drivetrain.getChassisSpeeds(), this.drivetrain.getPose().getRotation());
        target = drivetrain.getPoseToScore(this.drivetrain.getAngleToReefPolar());
        Logger.recordOutput("AutoSnap/PoseTarget", this.target);
        Translation2d translation2d = this.drivetrain.getPose().getTranslation();

        // Reset the PID controllers with the current position and velocity.
        this.vx.reset(translation2d.getX(), fieldRelativeSpeeds.vxMetersPerSecond);
        this.vy.reset(translation2d.getY(), fieldRelativeSpeeds.vyMetersPerSecond);

        runAutoSnap();
    }

    @Override
    public void execute() {
        runAutoSnap();
    }

    @Override
    public void end(boolean interrupted) {
        this.drivetrain.setControl(this.drivetrain.m_robotSpeeds.withSpeeds(new ChassisSpeeds()));
    }

    @Override
    public boolean isFinished() {
        return this.vx.atGoal() && this.vy.atGoal();
    }


    private void runAutoSnap() {
        Translation2d translation2d = this.drivetrain.getPose().getTranslation();

        // Add the current setpoint velocity to the new velocity so that it is smoother.
        double outputX = this.vx.calculate(translation2d.getX(), this.target.getX()) + this.vx.getSetpoint().velocity;
        double outputY = this.vy.calculate(translation2d.getY(), this.target.getY()) + this.vy.getSetpoint().velocity;

        // If it is not scaled, then each controller can output speeds that when added up to the final vector
        // will exceed the maximum real speed of the robot.
        outputX *= 0.7;
        outputY *= 0.7;

        Logger.recordOutput("AutoSnap/outputX", outputX);
        Logger.recordOutput("AutoSnap/outputY", outputY);
        Logger.recordOutput("AutoSnap/vxAtGoal", this.vx.atGoal());
        Logger.recordOutput("AutoSnap/vyAtGoal", this.vy.atGoal());


        this.drivetrain.setControl(
                this.driveHeading
                        .withVelocityY(outputY)
                        .withVelocityX(outputX)
                        .withTargetDirection(this.target.getRotation())
        );
    }
}
