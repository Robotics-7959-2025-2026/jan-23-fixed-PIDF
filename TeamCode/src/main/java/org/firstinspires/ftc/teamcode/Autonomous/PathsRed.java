package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

public class PathsRed extends Paths {
    public static final double maxRotTimeGrab = 0.5;
    public static final double maxRotTimeShoot = 0.8;
    
    public PathsRed(Follower follower) {
        shootPre = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(122.500, 122.000), new Pose(85.000, 90.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(36), maxRotTimeShoot)
                .build();

        grabOne = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(85.000, 90.000),
                                new Pose(84.000, 84.000),
                                new Pose(118.500, 84.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(0), maxRotTimeGrab)
                .build();

        shootOne = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(118.500, 84.000), new Pose(85.000, 90.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(36), maxRotTimeShoot)
                .build();

        grabTwo = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(85.000, 90.000),
                                new Pose(84.000, 58.000),
                                new Pose(96.000, 60.000),
                                new Pose(116.000, 60.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(0), maxRotTimeGrab)
                .build();

        shootTwo = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(116.000, 60.000), new Pose(85.000, 90.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(36), maxRotTimeShoot)
                .build();

        grabThree = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(85.000, 90.000),
                                new Pose(84.000, 66.000),
                                new Pose(84.000, 32.000),
                                new Pose(96.000, 36.000),
                                new Pose(119.000, 36.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(0), maxRotTimeGrab)
                .build();

        shootThree = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(119.000, 36.000), new Pose(85.000, 90.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(36), maxRotTimeShoot)
                .build();

        hitLever = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(85.000, 90.000),
                                new Pose(96.000, 72.000),
                                new Pose(123.000, 67.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(0), maxRotTimeGrab)
                .build();
        startPose = new Pose(122.500, 122.000, Math.toRadians(36));
    }
}
