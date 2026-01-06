package frc.robot.commands.autos;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.lib.BLine.Path;

public class DriveForward {
    private final Command followCommand;

    public DriveForward(RobotContainer robotContainer) {
        var drive = robotContainer.getDrive();

        Path myPath =
                new Path(
                        new Path.Waypoint(new Translation2d(1.0, 1.0), new Rotation2d(0)),
                        new Path.TranslationTarget(new Translation2d(2.0, 2.0)),
                        new Path.Waypoint(new Translation2d(3.0, 1.0), new Rotation2d(Math.PI)));

        // Use your builder from the container (rename to match your RobotContainer API)
        // Example assumes RobotContainer exposes a FollowPath.Builder via a getter.
        followCommand = drive.getPathBuilder().build(myPath);
    }

    public Command getCommand() {
        return followCommand;
    }
}
