package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Shooter;

@TeleOp(name="teleeop11")
public class Teleeop11 extends LinearOpMode {
    private Robot robot;
    private Intake intake;
    private Shooter shooter;
    private Follower follower;

    @Override
    public void runOpMode() {
        robot=new Robot(hardwareMap);
        intake=new Intake(hardwareMap);
        shooter=new Shooter(hardwareMap);
        follower=Constants.createFollower(hardwareMap);

        telemetry.addLine("teleeop11");
        telemetry.addLine("robot centric");
        telemetry.update();

        waitForStart();
        follower.update();

        while(opModeIsActive()) {
            follower.localizer.update();

            double y=stick(-gamepad1.left_stick_y);
            double x=stick(gamepad1.left_stick_x);
            double rx=stick(gamepad1.right_stick_x);

            double speed=Constants.DRIVE_SPEED;

            if(gamepad1.left_stick_button) {
                speed=Constants.SLOW_SPEED;
            }

            double turn=rx*speed;

            robot.drive.drive(y,x,turn,speed);

            if(gamepad2.a) {
                intake.setStopper_open();
            }else {
                intake.setStopper_close();
            }

            if(gamepad2.right_bumper) {
                intake.in();
            }else if(gamepad2.left_bumper) {
                intake.out();
            }else {
                intake.stop();
            }

            if(gamepad2.left_trigger>0.3) {
                shooter.setTargetVelocity(Constants.SHOOTER_TARGET_VELOCITY);
            }else {
                shooter.setTargetVelocity(0);
            }
            shooter.update();

            telemetry.addData("X",follower.pose().x());
            telemetry.addData("Y",follower.pose().y());
            telemetry.addData("Heading",Math.toDegrees(follower.pose().heading()));
            telemetry.addData("Shooter",shooter.getPower());
            telemetry.addData("Shooter target",shooter.getTargetVelocity());
            telemetry.addData("Shooter speed",shooter.getSpeed());
            telemetry.addData("Shooter L",shooter.getLeftSpeed());
            telemetry.addData("Shooter R",shooter.getRightSpeed());
            telemetry.update();
        }
    }

    private double stick(double value) {
        if(Math.abs(value)<Constants.STICK_DEADZONE) {
            return 0;
        }
        return value*value*value;
    }
}