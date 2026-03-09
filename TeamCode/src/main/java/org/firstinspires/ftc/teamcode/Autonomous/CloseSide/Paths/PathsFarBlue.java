package org.firstinspires.ftc.teamcode.Autonomous.CloseSide.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

public class PathsFarBlue extends Paths {
    public static final double maxRotTimeGrab = 0.4;
    public static final double maxRotTimeShoot = 1;

    public PathsFarBlue(Follower follower) {
        shootPre = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(56.771, 8.520),
                                new Pose(57.465, 12.405)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(114), maxRotTimeShoot)
                .build();

        grabTwo = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(57.465, 12.405),
                                new Pose(59.569, 40.166),
                                new Pose(15.788, 33.484)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(114), Math.toRadians(180), maxRotTimeGrab)
                .build();

        shootTwo = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(15.788, 33.484),
                                new Pose(57.552, 12.311)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(114), maxRotTimeShoot)
                .build();

        grabOne = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(57.552, 12.311),
                                new Pose(31.524, 17.875),
                                new Pose(33.305, 8.716),
                                new Pose(11.058, 9.120)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(114), Math.toRadians(180), maxRotTimeGrab)
                .build();

        shootOne = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(11.058, 9.120),
                                new Pose(37.625, 24.367),
                                new Pose(57.610, 12.304)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(114), maxRotTimeShoot)
                .build();

        grabThree = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(57.610, 12.304),
                                new Pose(29.993, 17.022),
                                new Pose(31.345, 8.229),
                                new Pose(10.978, 9.687)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(114), Math.toRadians(180), maxRotTimeGrab)
                .build();

        shootThree = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(10.978, 9.687),
                                new Pose(26.699, 18.886),
                                new Pose(57.504, 12.398)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(114), maxRotTimeShoot)
                .build();

        leaveZone = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(57.504, 12.398),
                                new Pose(36.482, 15.860)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(114))
                .build();
        startPose = new Pose(56.77108433734941, 8.520481927710843, Math.toRadians(90));
    }
}
