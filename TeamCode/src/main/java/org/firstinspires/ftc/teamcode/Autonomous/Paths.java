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

    // GOD I wish we had Rust construction semantics
    public static Paths blue(Follower follower) {
        Paths paths = new Paths();
        paths.shootPre = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(21.500, 122.000), new Pose(59.000, 90.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(144), 0.75)
                .build();

        paths.grabOne = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(59.000, 90.000),
                                new Pose(60.000, 84.000),
                                new Pose(18.000, 84.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180), 0.75)
                .build();

        paths.shootOne = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(18.000, 84.000), new Pose(59.000, 90.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(144), 0.75)
                .build();

        paths.grabTwo = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(59.000, 90.000),
                                new Pose(60.000, 58.000),
                                new Pose(48.000, 60.000),
                                new Pose(18.000, 60.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180), 0.75)
                .build();

        paths.shootTwo = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(18.000, 60.000), new Pose(59.000, 90.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(144), 0.75)
                .build();

        paths.grabThree = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(59.000, 90.000),
                                new Pose(60.000, 66.000),
                                new Pose(60.000, 32.000),
                                new Pose(48.000, 36.000),
                                new Pose(18.000, 36.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180), 0.75)
                .build();

        paths.shootThree = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(18.000, 36.000), new Pose(59.000, 90.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(144), 0.75)
                .build();

        paths.hitLever = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(59.000, 90.000),
                                new Pose(48.000, 72.000),
                                new Pose(17.000, 72.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180), 0.75)
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
                .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(36), 0.75)
                .build();

        paths.grabOne = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(85.000, 90.000),
                                new Pose(84.000, 84.000),
                                new Pose(126.000, 84.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(0), 0.75)
                .build();

        paths.shootOne = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(126.000, 84.000), new Pose(85.000, 90.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(36), 0.75)
                .build();

        paths.grabTwo = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(85.000, 90.000),
                                new Pose(84.000, 58.000),
                                new Pose(96.000, 60.000),
                                new Pose(126.000, 60.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(0), 0.75)
                .build();

        paths.shootTwo = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(126.000, 60.000), new Pose(85.000, 90.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(36), 0.75)
                .build();

        paths.grabThree = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(85.000, 90.000),
                                new Pose(84.000, 66.000),
                                new Pose(84.000, 32.000),
                                new Pose(96.000, 36.000),
                                new Pose(126.000, 36.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(0), 0.75)
                .build();

        paths.shootThree = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(126.000, 36.000), new Pose(85.000, 90.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(36), 0.75)
                .build();

        paths.hitLever = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(85.000, 90.000),
                                new Pose(96.000, 72.000),
                                new Pose(127.000, 72.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(0), 0.75)
                .build();
        return paths;
    }

    public static Pose redStartPose() {
        return new Pose(122.500, 122.000, Math.toRadians(36));
    }
}
