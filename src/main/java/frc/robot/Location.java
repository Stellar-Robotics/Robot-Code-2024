package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public enum Location {
    AMP(new Pose2d(0, 0, new Rotation2d(0))),
    STAGE(new Pose2d(4, 4, new Rotation2d(0)));

    public final Pose2d pose;
    
    private Location(Pose2d pose) {
        this.pose = pose;
    }
}
