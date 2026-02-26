package org.firstinspires.ftc.teamcode.Autonomous.FarSide.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Autonomous.FarSide.Paths.FarPaths;

public class FarPathsBlue extends FarPaths {
    public static final double maxRotTimeGrab = 0.5;
    public static final double maxRotTimeShoot = 0.8;

    public FarPathsBlue(Follower follower) {
        shootPre = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(64.000, 8.000),

                                new Pose(60.098, 22.537)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(110))

                .build();

        grabOne = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(60.098, 22.537),
                                new Pose(48.573, 11.720),
                                new Pose(13.439, 14.951)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(-150))

                .build();

        shootOne = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(13.439, 14.951),
                                new Pose(42.317, 11.963),
                                new Pose(59.878, 22.585)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-150), Math.toRadians(110))

                .build();

        grabTwo = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(59.878, 22.585),
                                new Pose(53.463, 6.207),
                                new Pose(19.244, 11.634)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(180))

                .build();

        shootTwo = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(19.244, 11.634),
                                new Pose(53.378, 6.732),
                                new Pose(60.293, 22.707)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(110))

                .build();

        leaveZone = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(60.293, 22.707),

                                new Pose(44.122, 13.878)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(110))

                .build();
        startPose = new Pose(64, 8, Math.toRadians(90));
    }
}
