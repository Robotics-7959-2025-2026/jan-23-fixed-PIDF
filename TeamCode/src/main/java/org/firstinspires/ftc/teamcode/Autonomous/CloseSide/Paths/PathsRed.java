package org.firstinspires.ftc.teamcode.Autonomous.CloseSide.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

public class PathsRed extends Paths {
    public static final double maxRotTimeGrab = 0.5;
    public static final double maxRotTimeShoot = 0.8;
    
    public PathsRed(Follower follower) {
        shootPre = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(122.500, 122.000),

                                new Pose(86.146, 84.780)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(42))

                .build();

        grabTwo = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(86.146, 84.780),
                                new Pose(88.183, 57.707),
                                new Pose(84.963, 58.683),
                                new Pose(125.146, 58.878)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(0))

                .build();

        hitLever = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(125.146, 58.878),
                                new Pose(110.049, 66.537),
                                new Pose(123.610, 62.146)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();

        shootTwo = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(123.610, 62.146),
                                new Pose(84.476, 56.390),
                                new Pose(86.463, 84.780)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(42))

                .build();

        grabOne = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(86.463, 84.780),

                                new Pose(123.488, 83.537)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(41), Math.toRadians(0))

                .build();

        shootOne = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(123.488, 83.537),

                                new Pose(86.000, 84.683)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(42))

                .build();

        grabThree = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(86.000, 84.683),
                                new Pose(75.951, 21.244),
                                new Pose(120.488, 37.268)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(42), Math.toRadians(0))

                .build();

        shootThree = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(120.488, 37.268),

                                new Pose(85.610, 84.951)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(42))

                .build();

        leaveZone = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(85.610, 84.951),

                                new Pose(94.976, 72.878)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(42), Math.toRadians(42))

                .build();
        startPose = new Pose(122.500, 122.000, Math.toRadians(36));
    }
}
