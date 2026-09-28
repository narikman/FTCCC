package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Blue Parking Test")
public class autoblue extends AutoBase {

    private static final double START_X=89.74328859060407;
    private static final double START_Y=136.98909395973150;
    private static final double START_H=90;

    private static final double PARK_X=135.1552438;
    private static final double PARK_Y=27.56711409;
    private static final double PARK_H=0;

    @Override
    public void runOpMode() {
        initRobot();

        follower.setPose(new Pose(
                START_X,
                START_Y,
                Math.toRadians(START_H)
        ));

        telemetry.addLine("Blue Parking Test");
        telemetry.addData("Start X",START_X);
        telemetry.addData("Start Y",START_Y);
        telemetry.update();

        waitForStart();

        if(isStopRequested()) return;

        followLine(PARK_X,PARK_Y,PARK_H);

        while(opModeIsActive()&&follower.isBusy()) {
            follower.update();

            telemetry.addData("X",follower.pose().x());
            telemetry.addData("Y",follower.pose().y());
            telemetry.addData(
                    "Heading",
                    Math.toDegrees(follower.pose().heading())
            );
            telemetry.addData("Busy",follower.isBusy());
            telemetry.update();
        }

        holdHere();

        while(opModeIsActive()) {
            follower.update();

            telemetry.addLine("Parked");
            telemetry.addData("X",follower.pose().x());
            telemetry.addData("Y",follower.pose().y());
            telemetry.update();
        }
    }
}