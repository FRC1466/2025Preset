// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;

public class DriveToPoseFusion extends SequentialCommandGroup {
  private static final PathConstraints constraints = new PathConstraints(4, 3, 360.0, 720.0);

  /**
   * Creates a new DriveToPoseFusion command. This command first pathfinds to the target pose using
   * AutoBuilder, and then switches to a local PID loop (DriveToPose) to lock in the final pose.
   *
   * @param drive The drive subsystem.
   * @param targetPose A supplier for the target pose.
   * @param constraints The path constraints for pathfinding.
   */
  public DriveToPoseFusion(Drive drive, Supplier<Pose2d> targetPose, PathConstraints constraints) {
    addCommands(
        // Use drive.defer() to evaluate the command sequence at runtime
        drive.defer(
            () ->
                new SequentialCommandGroup(
                    // First: PathPlanner pathfinding to get close to target
                    AutoBuilder.pathfindToPose(targetPose.get(), constraints),
                    // Then: Local PID control for precise final positioning
                    new DriveToPose(drive, targetPose))));
  }

  /**
   * Creates a new DriveToPoseFusion command. This command first pathfinds to the target pose using
   * AutoBuilder, and then switches to a local PID loop (DriveToPose) to lock in the final pose.
   *
   * @param drive The drive subsystem.
   * @param targetPose A supplier for the target pose.
   */
  public DriveToPoseFusion(Drive drive, Supplier<Pose2d> targetPose) {
    this(drive, targetPose, constraints);
  }
}
