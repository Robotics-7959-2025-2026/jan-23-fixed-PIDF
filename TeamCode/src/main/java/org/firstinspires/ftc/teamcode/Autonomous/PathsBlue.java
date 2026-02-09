package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

public class PathsBlue extends Paths {
    public static final double maxRotTimeGrab = 0.5;
    public static final double maxRotTimeShoot = 0.8;
    
    public PathsBlue(Follower follower) {
        shootPre = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(21.500, 122.000), new Pose(59.000, 90.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(144), maxRotTimeShoot)
                .build();

        grabOne = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(59.000, 90.000),
                                new Pose(60.000, 84.000),
                                new Pose(19.000, 84.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180), maxRotTimeGrab)
                .build();

        shootOne = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(19.000, 84.000), new Pose(59.000, 90.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(144), maxRotTimeShoot)
                .build();

        grabTwo = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(59.000, 90.000),
                                new Pose(60.000, 58.000),
                                new Pose(48.000, 60.000),
                                new Pose(19.000, 60.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180), maxRotTimeGrab)
                .build();

        shootTwo = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(19.000, 60.000), new Pose(59.000, 90.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(144), maxRotTimeShoot)
                .build();

        grabThree = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(59.000, 90.000),
                                new Pose(60.000, 66.000),
                                new Pose(60.000, 32.000),
                                new Pose(48.000, 36.000),
                                new Pose(19.000, 36.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180), maxRotTimeGrab)
                .build();

        shootThree = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(18.000, 36.000), new Pose(59.000, 90.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(144), maxRotTimeShoot)
                .build();

        hitLever = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(59.000, 90.000),
                                new Pose(48.000, 72.000),
                                new Pose(16.000, 72.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180), maxRotTimeGrab)
                .build();
        startPose = new Pose(21.500, 122.000, Math.toRadians(144));
    }
}
