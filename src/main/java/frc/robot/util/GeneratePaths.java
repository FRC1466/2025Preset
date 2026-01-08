// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466

package frc.robot.util;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.List;

public class GeneratePaths {

  // Pre-generate all paths at class initialization
  private static final PathPlannerPath CIRCLE_PATH = generateCirclePath();

  /**
   * Gets the pre-generated circle path.
   *
   * @return The circle PathPlannerPath.
   */
  public static PathPlannerPath getCirclePath() {
    return CIRCLE_PATH;
  }

  /**
   * Generates a circular path with 8 waypoints. Called once at class initialization.
   *
   * @return A PathPlannerPath object forming a circle.
   */
  private static PathPlannerPath generateCirclePath() {
    double radius = 2.0; // Circle radius in meters
    double centerX = 3.0;
    double centerY = 3.0;
    int numPoints = 8;

    // Create waypoints in a circle
    Pose2d[] poses = new Pose2d[numPoints];
    for (int i = 0; i < numPoints; i++) {
      double angle = 2 * Math.PI * i / numPoints;
      double x = centerX + radius * Math.cos(angle);
      double y = centerY + radius * Math.sin(angle);
      // Direction of travel is tangent to the circle
      Rotation2d heading = Rotation2d.fromRadians(angle + Math.PI / 2);
      poses[i] = new Pose2d(x, y, heading);
    }

    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(poses);

    PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI);

    // Create the path using the waypoints created above
    return new PathPlannerPath(
        waypoints,
        constraints,
        null, // The ideal starting state, only relevant for pre-planned paths
        new GoalEndState(0.0, poses[0].getRotation()) // End at same rotation as start
        );
  }

  /**
   * Generates a custom path from a list of poses. Use this method to create paths dynamically at
   * runtime if needed.
   *
   * @param poses List of poses to create waypoints from.
   * @param constraints Path constraints for the generated path.
   * @param endVelocity Desired velocity at the end of the path.
   * @param endRotation Desired holonomic rotation at the end of the path.
   * @return A PathPlannerPath object.
   */
  public static PathPlannerPath generatePath(
      List<Pose2d> poses, PathConstraints constraints, double endVelocity, Rotation2d endRotation) {

    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(poses.toArray(new Pose2d[0]));

    return new PathPlannerPath(
        waypoints, constraints, null, new GoalEndState(endVelocity, endRotation));
  }
}
