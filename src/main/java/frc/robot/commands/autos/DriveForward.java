// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466

package frc.robot.commands.autos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class DriveForward {
  private final Command followCommand;

  public DriveForward(RobotContainer robotContainer, DoubleSupplier P, DoubleSupplier D) {
    Drive drive = robotContainer.getDrive();

    Pose2d[] points =
        new Pose2d[] {
          new Pose2d(1.0, 1.0, new Rotation2d(0.0)),
          new Pose2d(2.0, 2.0, new Rotation2d(0.0)), // no rotation provided for TranslationTarget
          new Pose2d(3.0, 1.0, new Rotation2d(Math.PI))
        };

    Logger.recordOutput("Autos/DriveForward/Points", posesToLogArray(points));

    Path myPath =
        new Path(
            new Path.Waypoint(new Translation2d(1.0, 1.0), new Rotation2d(0)),
            new Path.TranslationTarget(new Translation2d(2.0, 2.0)),
            new Path.Waypoint(new Translation2d(3.0, 1.0), new Rotation2d(Math.PI)));

    FollowPath.Builder pathBuilder =
        new FollowPath.Builder(
                robotContainer.getDrive(),
                robotContainer.getDrive()::getPose,
                robotContainer.getDrive()::getChassisSpeeds,
                robotContainer.getDrive()::runVelocity,
                new PIDController(P.getAsDouble(), 0, D.getAsDouble()), // translation
                new PIDController(0, 0.0, 0.0), // rotation
                new PIDController(0, 0.0, 0.0) // cross-track
                )
            .withDefaultShouldFlip()
            .withPoseReset(robotContainer.getDrive()::setPose);

    followCommand = pathBuilder.build(myPath);
  }

  public Command getCommand() {
    return followCommand;
  }

  private static double[] posesToLogArray(Pose2d[] poses) {
    double[] out = new double[poses.length * 3];
    for (int i = 0; i < poses.length; i++) {
      Pose2d p = poses[i];
      out[i * 3] = p.getX();
      out[i * 3 + 1] = p.getY();
      out[i * 3 + 2] = p.getRotation().getRadians();
    }
    return out;
  }
}
