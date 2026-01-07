// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466

package frc.robot.util;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;

public class AllianceFlipUtil {
  public static double applyX(double x) {
    return shouldFlip() ? Constants.fieldLengthMeters - x : x;
  }

  public static double applyY(double y) {
    return shouldFlip() ? Constants.fieldWidthMeters - y : y;
  }

  public static Rotation2d apply(Rotation2d rotation) {
    return shouldFlip() ? new Rotation2d(Math.PI).plus(rotation) : rotation;
  }

  public static boolean shouldFlip() {
    return DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red;
  }
}
