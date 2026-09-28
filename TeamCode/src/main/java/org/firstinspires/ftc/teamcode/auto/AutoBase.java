package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.AtomicPath;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.curves.Line;
import com.pedropathing.paths.curves.bezier.BezierCurve;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.robot.Robot;

public abstract class AutoBase extends LinearOpMode {
    protected Follower follower;
    protected Robot robot;

    protected void initRobot() {
        robot=new Robot(hardwareMap);
        follower=Constants.createFollower(hardwareMap);
    }

    protected void followCurve(double controlX,double controlY,double x,double y,double heading) {
        Pose start=follower.pose();
        Pose control=new Pose(
                controlX,
                controlY,
                start.heading()
        );
        Pose target=new Pose(
                x,
                y,
                Math.toRadians(heading)
        );

        Path path=new AtomicPath(
                new BezierCurve(start,control,target)
        ).linear(start,target);

        follower.follow(path);
    }

    protected void followLine(double x,double y,double heading) {
        Pose start=follower.pose();
        Pose target=new Pose(
                x,
                y,
                Math.toRadians(heading)
        );

        Path path=new AtomicPath(
                new Line(start,target)
        ).linear(start,target);

        follower.follow(path);
    }

    protected void holdHere() {
        follower.hold(follower.pose());
    }

    protected void turn(double degrees) {
        follower.hold(
                follower.pose().withHeading(
                        Math.toRadians(degrees)
                )
        );
    }
}