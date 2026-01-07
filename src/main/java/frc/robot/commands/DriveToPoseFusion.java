// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;
import java.util.Set;
import java.util.function.Supplier;

public class DriveToPoseFusion extends SequentialCommandGroup {

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
        // Phase 1: Use DeferredCommand to generate the pathfinding command at runtime.
        // This ensures the current robot pose is used as the starting point, not the
        // pose from when the robot turned on.
        new DeferredCommand(
            () -> AutoBuilder.pathfindToPoseFlipped(targetPose.get(), constraints), Set.of(drive)),

        // Phase 2: Use local PID controller to finalize and hold the pose
        new DriveToPose(drive, targetPose));
  }
}
