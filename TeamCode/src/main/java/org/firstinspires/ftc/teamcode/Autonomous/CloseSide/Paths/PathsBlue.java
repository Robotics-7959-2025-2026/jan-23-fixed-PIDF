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
                                new Pose(55.422, 87.733)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(137.5), maxRotTimeShoot)
                .build();

        grabTwo = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(55.422, 87.733),
                                new Pose(55.991, 59.792),
                                new Pose(57.126, 52.430),
                                new Pose(17.985, 60.441)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(137.5), Math.toRadians(180), maxRotTimeGrab)
                .build();

        shootTwo = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(17.985, 60.441),
                                new Pose(59.524, 56.390),
                                new Pose(55.105, 87.733)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(137.5), maxRotTimeShoot)
                .build();

        grabOne = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(55.105, 87.733),
                                new Pose(52.171, 55.141),
                                new Pose(10.618, 56.295)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(137.5), Math.toRadians(155), maxRotTimeGrab)
                .build();

        shootOne = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(10.618, 56.295),
                                new Pose(46.298, 63.975),
                                new Pose(55.221, 87.810)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(155), Math.toRadians(137.5), maxRotTimeShoot)
                .build();

        grabThree = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(55.221, 87.810),
                                new Pose(47.441, 79.798),
                                new Pose(18.480, 79.951)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(137.5), Math.toRadians(180), maxRotTimeGrab)
                .build();

        shootThree = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(18.480, 79.951),
                                new Pose(55.090, 87.904)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(137.5), maxRotTimeShoot)
                .build();

        leaveZone = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(55.090, 87.904),
                                new Pose(45.377, 74.441)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(137.5), Math.toRadians(137.5))
                .build();
        startPose = new Pose(21.500, 122.000, Math.toRadians(144));
    }
}
