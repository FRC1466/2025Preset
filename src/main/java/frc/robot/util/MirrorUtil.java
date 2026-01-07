// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import java.util.function.BooleanSupplier;

public class MirrorUtil {
  private static BooleanSupplier mirror;

  public static void setMirror(BooleanSupplier mirrorSupplier) {
    mirror = mirrorSupplier;
  }

  public static Pose2d apply(Pose2d pose) {
    if (!mirror.getAsBoolean()) return pose;
    return new Pose2d(
        pose.getX(),
        Constants.fieldWidthMeters - pose.getY(),
        new Rotation2d(pose.getRotation().getCos(), -pose.getRotation().getSin()));
  }
}
