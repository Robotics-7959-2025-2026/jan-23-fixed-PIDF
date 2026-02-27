package org.firstinspires.ftc.teamcode.Autonomous.FarSide.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Autonomous.FarSide.Paths.FarPaths;

public class FarPathsRed extends FarPaths {
    public static final double maxRotTimeGrab = 0.5;
    public static final double maxRotTimeShoot = 0.8;

    public FarPathsRed(Follower follower) {
        shootPre = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(80.000, 8.000),

                                new Pose(83.902, 22.537)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(66))

                .build();

        grabOne = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(83.902, 22.537),
                                new Pose(95.427, 11.720),
                                new Pose(130.561, 14.951)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(66), Math.toRadians(330))

                .build();

        shootOne = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(130.561, 14.951),
                                new Pose(101.683, 11.573),
                                new Pose(84.122, 22.585)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(330), Math.toRadians(66))

                .build();

        grabTwo = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(84.122, 22.585),
                                new Pose(95.610, 8.939),
                                new Pose(124.756, 11.634)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(66), Math.toRadians(0))

                .build();

        shootTwo = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(124.756, 11.634),
                                new Pose(96.085, 9.073),
                                new Pose(83.707, 22.707)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(63))

                .build();

        leaveZone = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(83.707, 22.707),
                                new Pose(93.793, 4.695),
                                new Pose(112.659, 8.976)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(63), Math.toRadians(0))

                .build();
        startPose = new Pose(80, 8, Math.toRadians(90));
    }
}
