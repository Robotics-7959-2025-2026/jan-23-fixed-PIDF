package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class Paths {
    public PathChain shootPre;
    public PathChain grabOne;
    public PathChain shootOne;
    public PathChain grabTwo;
    public PathChain shootTwo;
    public PathChain grabThree;
    public PathChain shootThree;
    public PathChain hitLever;

    public static final double maxRotTime = 0.5;

    // GOD I wish we had Rust construction semantics
    public static Paths blue(Follower follower) {
        Paths paths = new Paths();
        paths.shootPre = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(21.500, 122.000), new Pose(59.000, 90.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(144), maxRotTime)
                .build();

        paths.grabOne = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(59.000, 90.000),
                                new Pose(60.000, 84.000),
                                new Pose(19.000, 84.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180), maxRotTime)
                .build();

        paths.shootOne = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(19.000, 84.000), new Pose(59.000, 90.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(144), maxRotTime)
                .build();

        paths.grabTwo = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(59.000, 90.000),
                                new Pose(60.000, 58.000),
                                new Pose(48.000, 60.000),
                                new Pose(19.000, 60.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180), maxRotTime)
                .build();

        paths.shootTwo = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(19.000, 60.000), new Pose(59.000, 90.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(144), maxRotTime)
                .build();

        paths.grabThree = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(59.000, 90.000),
                                new Pose(60.000, 66.000),
                                new Pose(60.000, 32.000),
                                new Pose(48.000, 36.000),
                                new Pose(19.000, 36.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180), maxRotTime)
                .build();

        paths.shootThree = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(18.000, 36.000), new Pose(59.000, 90.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(144), maxRotTime)
                .build();

        paths.hitLever = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(59.000, 90.000),
                                new Pose(48.000, 72.000),
                                new Pose(16.000, 72.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180), maxRotTime)
                .build();
        return paths;
    }

    public static Pose blueStartPose() {
        return new Pose(21.500, 122.000, Math.toRadians(144));
    }

    public static Paths red(Follower follower) {
        Paths paths = new Paths();
        paths.shootPre = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(122.500, 122.000), new Pose(85.000, 90.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(36), maxRotTime)
                .build();

        paths.grabOne = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(85.000, 90.000),
                                new Pose(84.000, 84.000),
                                new Pose(118.500, 84.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(0), maxRotTime)
                .build();

        paths.shootOne = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(118.500, 84.000), new Pose(85.000, 90.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(36), maxRotTime)
                .build();

        paths.grabTwo = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(85.000, 90.000),
                                new Pose(84.000, 58.000),
                                new Pose(96.000, 60.000),
                                new Pose(116.000, 60.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(0), maxRotTime)
                .build();

        paths.shootTwo = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(116.000, 60.000), new Pose(85.000, 90.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(36), maxRotTime)
                .build();

        paths.grabThree = follower
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
                .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(0), maxRotTime)
                .build();

        paths.shootThree = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(119.000, 36.000), new Pose(85.000, 90.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(36), maxRotTime)
                .build();

        paths.hitLever = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(85.000, 90.000),
                                new Pose(96.000, 72.000),
                                new Pose(123.000, 67.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(0), maxRotTime)
                .build();
        return paths;
    }

    public static Pose redStartPose() {
        return new Pose(122.500, 122.000, Math.toRadians(36));
    }
}
