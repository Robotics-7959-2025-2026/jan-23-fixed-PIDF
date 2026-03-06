package org.firstinspires.ftc.teamcode.Autonomous.CloseSide.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

public class PathsFarRed extends Paths {
    public static final double maxRotTimeGrab = 0.5;
    public static final double maxRotTimeShoot = 0.8;

    public PathsFarRed(Follower follower) {
        shootPre = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(87.229, 8.520),
                                new Pose(86.535, 12.405)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(66),maxRotTimeShoot)
                .build();

        grabTwo = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(86.535, 12.405),
                                new Pose(84.431, 40.166),
                                new Pose(128.212, 33.484)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(66), Math.toRadians(0),maxRotTimeGrab)
                .build();

        shootTwo = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(128.212, 33.484),
                                new Pose(86.448, 12.311)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(66),maxRotTimeShoot)
                .build();

        grabOne = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(86.448, 12.311),
                                new Pose(112.476, 17.875),
                                new Pose(132.942, 9.120)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(66), Math.toRadians(0),maxRotTimeGrab)
                .build();

        shootOne = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(132.942, 9.120),
                                new Pose(106.375, 24.367),
                                new Pose(86.390, 12.304)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(66),maxRotTimeShoot)
                .build();

        grabThree = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(86.390, 12.304),
                                new Pose(114.007, 17.022),
                                new Pose(133.022, 9.687)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(66), Math.toRadians(0),maxRotTimeGrab)
                .build();

        shootThree = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(133.022, 9.687),
                                new Pose(117.301, 18.886),
                                new Pose(86.496, 12.398)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(66),maxRotTimeShoot)
                .build();

        leaveZone = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(86.496, 12.398),
                                new Pose(107.518, 15.860)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(66))
                .build();
        startPose = new Pose(87.22891566265059, 8.520481927710843, Math.toRadians(90));
    }
}

