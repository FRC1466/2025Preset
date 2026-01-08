// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;

public class PathfindThenFollow extends SequentialCommandGroup {

  /**
   * Creates a command that pathfinds to the start of a path, then follows that path.
   *
   * @param drive The drive subsystem.
   * @param pathSupplier A supplier for the path to follow.
   * @param constraints The pathfinding constraints.
   */
  public PathfindThenFollow(
      Drive drive, Supplier<PathPlannerPath> pathSupplier, PathConstraints constraints) {

    addCommands(
        // Use drive.defer() to evaluate the pathfinding command at runtime
        drive.defer(() -> AutoBuilder.pathfindThenFollowPath(pathSupplier.get(), constraints)));
  }

  /**
   * Creates a command that pathfinds to the start of a path, then follows that path. Uses default
   * constraints.
   *
   * @param drive The drive subsystem.
   * @param pathSupplier A supplier for the path to follow.
   */
  public PathfindThenFollow(Drive drive, Supplier<PathPlannerPath> pathSupplier) {
    this(
        drive,
        pathSupplier,
        new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720)));
  }
}
