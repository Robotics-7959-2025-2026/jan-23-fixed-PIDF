package org.firstinspires.ftc.teamcode.Autonomous.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

public class PathsBlue extends Paths {
    public static final double maxRotTimeGrab = 0.5;
    public static final double maxRotTimeShoot = 0.8;
    
    public PathsBlue(Follower follower) {
        shootPre = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(21.500, 122.000),

                                new Pose(59.000, 90.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(144))

                .build();

        grabOne = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(59.000, 90.000),
                                new Pose(60.000, 84.000),
                                new Pose(26.000, 84.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))

                .build();

        shootOne = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(26.000, 84.000),

                                new Pose(59.000, 90.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(144))

                .build();

        grabTwo = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(59.000, 90.000),
                                new Pose(60.000, 58.000),
                                new Pose(48.000, 60.000),
                                new Pose(28.000, 60.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))

                .build();

        shootTwo = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(28.000, 60.000),

                                new Pose(59.000, 90.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(144))

                .build();

        grabThree = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(59.000, 90.000),
                                new Pose(60.000, 66.000),
                                new Pose(60.000, 32.000),
                                new Pose(48.000, 36.000),
                                new Pose(25.000, 36.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))

                .build();

        shootThree = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(25.000, 36.000),

                                new Pose(59.000, 90.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(144))

                .build();

        hitLever = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(59.000, 90.000),
                                new Pose(41.111, 58.726),
                                new Pose(21.000, 72.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(165))

                .build();

        hitLeverReturn = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(21.000, 72.000),
                                new Pose(39.471, 61.812),
                                new Pose(59.000, 90.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(165), Math.toRadians(144))

                .build();

        leaveZone = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(59.000, 90.000),

                                new Pose(49.073, 79.122)
                        )
                ).setTangentHeadingInterpolation()
                .build();
        startPose = new Pose(21.500, 122.000, Math.toRadians(144));
    }
}
