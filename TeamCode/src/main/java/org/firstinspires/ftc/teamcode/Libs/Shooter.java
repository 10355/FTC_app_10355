package org.firstinspires.ftc.teamcode.Libs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.HardwareProfiles.HardwareTestPlatform;

/**
 * Created by caseyzandbergen on 12/1/16.
 */

public class Shooter {
    private HardwareTestPlatform robot = null;
    private LinearOpMode opMode = null;
    private boolean shooterLimit;
    private int shooterEncoderPosition;
    private int getShooterEncoderTarget;


    public Shooter(HardwareTestPlatform myRobot, LinearOpMode myOpMode) {
        robot = myRobot;
        opMode = myOpMode;
    }

    public int initShooter() {
        int target;
        while (!robot.touchSensor.isPressed()) {
            robot.motorShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorShooter.setPower(.2);
            shooterEncoderPosition = robot.motorShooter.getCurrentPosition();

            opMode.telemetry.addData("Limit Switch", String.valueOf(robot.touchSensor.isPressed()));
            opMode.telemetry.addData("ShooterEncoderPosition", String.valueOf(shooterEncoderPosition));
            opMode.telemetry.update();
        }
        target = robot.motorShooter.getCurrentPosition();
        while (shooterEncoderPosition > 0) {
            robot.motorShooter.setTargetPosition(0);
            robot.motorShooter.setPower(-.2);
            shooterEncoderPosition = robot.motorShooter.getCurrentPosition();

            opMode.telemetry.addData("Limit Switch", String.valueOf(robot.touchSensor.isPressed()));
            opMode.telemetry.addData("ShooterEncoderPosition", String.valueOf(shooterEncoderPosition));
            opMode.telemetry.update();
        }
        robot.motorShooter.setPower(0);
        return target;
    }

    public int cockShooter(int target) {
        shooterEncoderPosition = robot.motorShooter.getCurrentPosition();
        robot.motorShooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorShooter.setTargetPosition(target);
        robot.motorShooter.setPower(1);

        return shooterEncoderPosition;
    }

    public int shoot() {
        shooterEncoderPosition = robot.motorShooter.getCurrentPosition();
        robot.motorShooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        getShooterEncoderTarget = shooterEncoderPosition + 1440;
        robot.motorShooter.setTargetPosition(getShooterEncoderTarget);
        robot.motorShooter.setPower(1);

        shooterEncoderPosition = robot.motorShooter.getCurrentPosition();
        return shooterEncoderPosition;
    }

    public void feed(int time) {
        robot.motorFeeder.setPower(1);
        opMode.sleep(time);
    }

    public void feedStop() {
        robot.motorFeeder.setPower(0);
    }
}
