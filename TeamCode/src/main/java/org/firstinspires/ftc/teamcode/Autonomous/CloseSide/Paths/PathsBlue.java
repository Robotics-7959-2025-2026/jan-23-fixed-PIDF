package org.firstinspires.ftc.teamcode.Autonomous.CloseSide.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

public class PathsBlue extends Paths {
    public static final double maxRotTimeGrab = 0.5;
    public static final double maxRotTimeShoot = 0.8;
    
    public PathsBlue(Follower follower) {
        shootPre = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(21.500, 122.000),
                                new Pose(54.384, 86.689)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(138), maxRotTimeShoot)
                .build();

        grabTwo = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(54.384, 86.689),
                                new Pose(55.817, 57.707),
                                new Pose(59.037, 58.683),
                                new Pose(16.078, 56.449)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(180), maxRotTimeGrab)
                .build();

        shootTwo = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(16.078, 56.449),
                                new Pose(59.524, 56.390),
                                new Pose(54.067, 86.515)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(138), maxRotTimeShoot)
                .build();

        grabOne = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(54.067, 86.515),
                                new Pose(51.648, 53.062),
                                new Pose(11.664, 54.737)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(138), Math.toRadians(150), maxRotTimeGrab)
                .build();

        shootOne = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(11.664, 54.737),
                                new Pose(41.781, 67.623),
                                new Pose(54.183, 86.765)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(150), Math.toRadians(138), maxRotTimeShoot)
                .build();

        grabThree = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(54.183, 86.765),
                                new Pose(44.390, 74.814),
                                new Pose(16.225, 75.957)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(138), Math.toRadians(180), maxRotTimeGrab)
                .build();

        shootThree = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(16.225, 75.957),
                                new Pose(54.053, 86.686)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(138), maxRotTimeShoot)
                .build();

        leaveZone = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(54.053, 86.686),
                                new Pose(49.024, 72.878)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(138), Math.toRadians(138), maxRotTimeGrab)
                .build();
        startPose = new Pose(21.500, 122.000, Math.toRadians(144));
    }
}
