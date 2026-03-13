package org.firstinspires.ftc.teamcode.Autonomous.CloseSide.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

public class PathsRed extends Paths {
    public static final double maxRotTimeGrab = 0.5;
    public static final double maxRotTimeShoot = 0.8;
    
    public PathsRed(Follower follower) {
        shootPre = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(122.500, 122.000),
                                new Pose(86.667, 85.996)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(44), maxRotTimeShoot)
                .build();

        grabTwo = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(86.667, 85.996),
                                new Pose(88.009, 59.792),
                                new Pose(86.874, 52.430),
                                new Pose(124.973, 60.789)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(44), Math.toRadians(0), maxRotTimeGrab)
                .build();

        shootTwo = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(124.973, 60.789),
                                new Pose(84.476, 56.390),
                                new Pose(86.811, 86.170)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(44), maxRotTimeShoot)
                .build();

        grabOne = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(86.811, 86.170),
                                new Pose(93.739, 57.746),
                                new Pose(129.557, 56.986)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(44), Math.toRadians(22), maxRotTimeGrab)
                .build();

        shootOne = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(129.557, 56.986),
                                new Pose(97.702, 63.975),
                                new Pose(86.869, 86.073)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(22), Math.toRadians(44), maxRotTimeShoot)
                .build();

        grabThree = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(86.869, 86.073),
                                new Pose(96.559, 79.798),
                                new Pose(124.825, 79.256)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(44), Math.toRadians(0), maxRotTimeGrab)
                .build();

        shootThree = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(124.825, 79.256),
                                new Pose(86.999, 86.167)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(44), maxRotTimeShoot)
                .build();

        leaveZone = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(86.999, 86.167),
                                new Pose(98.623, 74.441)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(44), Math.toRadians(44))
                .build();
        startPose = new Pose(122.500, 122.000, Math.toRadians(36));
    }
}
