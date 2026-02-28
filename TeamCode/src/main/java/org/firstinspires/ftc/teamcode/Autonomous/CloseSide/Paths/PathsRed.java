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
                                new Pose(86.146, 84.780)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(42), maxRotTimeShoot)
                .build();

        grabTwo = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(86.146, 84.780),
                                new Pose(88.183, 57.707),
                                new Pose(84.963, 58.683),
                                new Pose(125.146, 59.572)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(0), maxRotTimeGrab)
                .build();

        shootTwo = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(125.146, 59.572),
                                new Pose(84.476, 56.390),
                                new Pose(86.463, 84.780)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(42), maxRotTimeShoot)
                .build();

        grabOne = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(86.463, 84.780),
                                new Pose(93.739, 57.746),
                                new Pose(129.213, 58.900)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(42), Math.toRadians(30), maxRotTimeGrab)
                .build();

        shootOne = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(129.213, 58.900),
                                new Pose(102.219, 67.623),
                                new Pose(86.000, 84.683)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(30), Math.toRadians(42), maxRotTimeShoot)
                .build();

        grabThree = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(86.000, 84.683),
                                new Pose(124.652, 77.345)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(42), Math.toRadians(0), maxRotTimeGrab)
                .build();

        shootThree = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(124.652, 77.345),
                                new Pose(85.610, 84.951)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(42), maxRotTimeShoot)
                .build();

        leaveZone = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(85.610, 84.951),
                                new Pose(94.976, 72.878)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(42), Math.toRadians(42), maxRotTimeGrab)
                .build();
        startPose = new Pose(122.500, 122.000, Math.toRadians(36));
    }
}
