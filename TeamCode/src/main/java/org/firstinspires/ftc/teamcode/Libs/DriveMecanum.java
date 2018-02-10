package org.firstinspires.ftc.teamcode.Libs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.teamcode.HardwareProfiles.HardwareTestPlatform;

import java.util.List;
import java.util.Locale;

/**
 * Created by caseyzandbergen on 12/7/16.
 * Foo
 */

public class DriveMecanum {
    public double myCurrentMotorPosition;
    public double myTargetPosition;
    public double LF, RF, LR, RR;
    public double robotX;          // The robot's X position from VuforiaLib
    public double robotY;  // The robot's Y position from VuforiaLib
    public double robotBearing;    //Bearing to, i.e. the bearing you need to stear toward
    public String procedure;
    public double initZ;
    public double currentZint;
    public double odsThreshold = .2;   //Threshold at which the ODS sensor acquires the whie line
    public double ods = 0;             //Value returned from the Optical Distance Sensor
    public double zCorrection = 0;
    private HardwareTestPlatform robot = null;
    private LinearOpMode opMode = null;
    private ElapsedTime runtime = new ElapsedTime();
    private DataLogger Dl;
    private double colorRightRed = 0;   //Value from color sensor
    private double colorRightBlue = 0;  //Value from color sensor
    private double colorLeftRed = 0;    //Value from color sensor
    private double colorLeftBlue = 0;   //Value from color sensor
    private double radians = 0;
    private double changeSpeed = 0;
    private double motorCorrectCoefficient = .05;
    private boolean tel = true;


    public DriveMecanum(HardwareTestPlatform myRobot, LinearOpMode myOpMode, DataLogger myDl) {
        robot = myRobot;
        opMode = myOpMode;
        Dl = myDl;

    }
    
    public void motorsHalt() {
        robot.motorLF.setPower(0);
        robot.motorRF.setPower(0);
        robot.motorLR.setPower(0);
        robot.motorRR.setPower(0);
    }

    /**
     * Set the power level of the motors.
     */
    public void setPower(double LF, double LR, double RF, double RR) {
        robot.motorLF.setPower(LF);
        robot.motorRF.setPower(RF);
        robot.motorLR.setPower(LR);
        robot.motorRR.setPower(RR);
    }

    /**
     * Pivot the robot to a new heading. 0 is straight ahead, 1 to 179 is to the left -1 to -179 is
     * to the right.
     */
    public void pivotLeft(double power, double heading) {
        procedure = "Pivot";
        initZ = robot.mrGyro.getIntegratedZValue();
        currentZint = initZ;

        while (currentZint < heading && opMode.opModeIsActive()) {
            /**
             * We are pivoting left so reverse power to the left motor
             */
            LR = -power;
            LF = -power;
            RR = power;
            RF = power;

            setPower(LF, LR, RF, RR);

            currentZint = robot.mrGyro.getIntegratedZValue();
            if (tel) {
                telemetry();
                logData();
            }
            opMode.idle();
            }
        motorsHalt();
    }

    public void pivotRight(double power, double heading) {
        procedure = "Pivot";
        initZ = robot.mrGyro.getIntegratedZValue();
        currentZint = robot.mrGyro.getIntegratedZValue();

        while (Math.abs(currentZint) < heading && opMode.opModeIsActive()) {
            /**
             * We are pivoting left so reverse power to the left motor
             */
            LR = power;
            LF = power;
            RR = -power;
            RF = -power;

            setPower(LF, LR, RF, RR);

            currentZint = robot.mrGyro.getIntegratedZValue();
            if (tel) {
                telemetry();
                logData();
            }
            opMode.idle();
        }
        motorsHalt();
    }

    /**
     * Translate on a heading and stop when the ODS sensor detects the white line.
     */
    public void translateOdsStop(double odsThreshold, double power, double heading) {
        procedure = "translateOdsStop";
        initZ = robot.mrGyro.getIntegratedZValue();
        ods = robot.ods.getLightDetected();

        radians = getRadians(heading);

        while (opMode.opModeIsActive() && ods < odsThreshold) {
            ods = robot.ods.getLightDetected();
            currentZint = robot.mrGyro.getIntegratedZValue();

            LF = calcLF(radians, power);
            RF = calcRF(radians, power);
            LR = calcLR(radians, power);
            RR = calcRR(radians, power);

            if (currentZint != initZ) {  //Robot has drifted off course
                zCorrection = Math.abs(initZ - currentZint);

                courseCorrect();
            } else {
                zCorrection = 0;
            }

            setPower(LF, LR, RF, RR);

            if (tel) {
                telemetry();
                logData();
            }

            opMode.idle();
        }

        motorsHalt();

    }

    /**
     * Translate on a heading the distance specified in MM.
     */
    public void translateDistance(double mm, double power, double heading) {
        procedure = "translateDistance";
        initZ = robot.mrGyro.getIntegratedZValue();
        myCurrentMotorPosition = robot.motorLF.getCurrentPosition();
        //myTargetPosition = myCurrentMotorPosition + (int) (mm * robot.COUNTS_PER_MM);

        radians = getRadians(heading);

        while (opMode.opModeIsActive() && myTargetPosition > myCurrentMotorPosition) {
            LF = calcLF(radians, power);
            RF = calcRF(radians, power);
            LR = calcLR(radians, power);
            RR = calcRR(radians, power);

            currentZint = robot.mrGyro.getIntegratedZValue();

            if (currentZint != initZ) {  //Robot has drifted off course
                zCorrection = Math.abs(initZ - currentZint);

                courseCorrect();
            } else {
                zCorrection = 0;
            }

            setPower(LF, LR, RF, RR);

            myCurrentMotorPosition = robot.motorLF.getCurrentPosition();

            if (tel) {
                telemetry();
                logData();
            }

            opMode.idle();
        }

        motorsHalt();

    }


    /**
     * Translate on a heading for a defined period of time.
     */
    public void translateRange(double power, double heading, int range) {
        double timeOutTime;
        procedure = "translateTime";
        initZ = robot.mrGyro.getIntegratedZValue();
        radians = getRadians(heading);

        while (opMode.opModeIsActive() &&
                robot.rangeSensor.rawUltrasonic() > range) {
            LF = calcLF(radians, power);
            RF = calcRF(radians, power);
            LR = calcLR(radians, power);
            RR = calcRR(radians, power);

            currentZint = robot.mrGyro.getIntegratedZValue();

            if (currentZint != initZ) {  //Robot has drifted off course
                zCorrection = Math.abs(initZ - currentZint);

                courseCorrect();
            } else {
                zCorrection = 0;
            }

            setPower(LF, LR, RF, RR);

            myCurrentMotorPosition = robot.motorLF.getCurrentPosition();

            if (tel) {
                telemetry();
                logData();
            }

            opMode.idle();
        }

        motorsHalt();

    }

    /**
     * Translate on a heading for a defined period of time.
     */
    public void translateRangeRight(double power, double heading, int range) {
        double timeOutTime;
        procedure = "translateRangeRight";
        initZ = robot.mrGyro.getIntegratedZValue();
        radians = getRadians(heading);

        while (opMode.opModeIsActive() &&
                robot.rangeSensorRight.rawUltrasonic() > range) {
            LF = calcLF(radians, power);
            RF = calcRF(radians, power);
            LR = calcLR(radians, power);
            RR = calcRR(radians, power);

            currentZint = robot.mrGyro.getIntegratedZValue();

            if (currentZint != initZ) {  //Robot has drifted off course
                zCorrection = Math.abs(initZ - currentZint);

                courseCorrect();
            } else {
                zCorrection = 0;
            }

            setPower(LF, LR, RF, RR);

            myCurrentMotorPosition = robot.motorLF.getCurrentPosition();

            if (tel) {
                telemetry();
                logData();
            }

            opMode.idle();
        }

        motorsHalt();

    }

    /**
     * Translate on a heading for a defined period of time.
     */
    public void translateTime(double timeOut, double power, double heading) {
        double timeOutTime;
        procedure = "translateTime";
        initZ = robot.mrGyro.getIntegratedZValue();
        radians = getRadians(heading);

        timeOutTime = runtime.time() + timeOut;

        while (opMode.opModeIsActive() && runtime.time() < timeOutTime) {
            LF = calcLF(radians, power);
            RF = calcRF(radians, power);
            LR = calcLR(radians, power);
            RR = calcRR(radians, power);

            opMode.telemetry.addData("range", robot.rangeSensor.rawUltrasonic());
            opMode.telemetry.update();

            currentZint = robot.mrGyro.getIntegratedZValue();

            if (currentZint != initZ) {  //Robot has drifted off course
                zCorrection = Math.abs(initZ - currentZint);

                courseCorrect();
            } else {
                zCorrection = 0;
            }

            setPower(LF, LR, RF, RR);

            myCurrentMotorPosition = robot.motorLF.getCurrentPosition();

            if (tel) {
                telemetry();
                logData();
            }

            opMode.idle();
        }

        motorsHalt();

    }

    /**
     * Calculate the wheel speeds.
     *
     * @return wheel speed
     */
    public double calcLF(double radians, double power) {
        LF = power * Math.sin(radians + (Math.PI / 4)) + changeSpeed;

        if (LF > 1 || LF < -1) {
            LF = 0;
        }

        return LF;
    }

    public double calcRF(double radians, double power) {
        RF = power * Math.cos(radians + (Math.PI / 4)) - changeSpeed;

        if (RF > 1 || RF < -1) {
            RF = 0;
        }

        return RF;
    }

    public double calcLR(double radians, double power) {
        LR = power * Math.cos(radians + (Math.PI / 4)) + changeSpeed;

        if (LR > 1 || LR < -1) {
            LR = 0;
        }

        return LR;
    }

    public double calcRR(double radians, double power) {
        RR = power * Math.sin(radians + (Math.PI / 4)) - changeSpeed;

        if (RR > 1 || RR < -1) {
            RR = 0;
        }

        return RR;
    }

    /**
     * Change power to the motors to correct for heading drift as indicated by the gyro.
     */
    public void courseCorrect() {
        if (currentZint > initZ) {  //Robot has drifted left
            LF = LF + motorCorrectCoefficient;
            LR = LR + motorCorrectCoefficient;
            RF = RF - motorCorrectCoefficient;
            RR = RR - motorCorrectCoefficient;
        }

        if (currentZint < initZ) {  //Robot has drifted right
            LF = LF - motorCorrectCoefficient;
            LR = LR - motorCorrectCoefficient;
            RF = RF + motorCorrectCoefficient;
            RR = RR + motorCorrectCoefficient;
        }
    }

    public double getRadians(double heading) {
        radians = heading * (Math.PI / 180);

        return radians;
    }

    /**
     * Transmit telemetry.
     */
    private void telemetry() {

        opMode.telemetry.addData("Procedure", String.valueOf(procedure));
        opMode.telemetry.addData("Current Z Int", String.valueOf(currentZint));
        opMode.telemetry.addData("Z Correction", String.valueOf(zCorrection));
        opMode.telemetry.addData("Range", String.valueOf(robot.rangeSensor.rawUltrasonic()));
        opMode.telemetry.addData("RangeRight", String.valueOf(robot.rangeSensorRight.rawUltrasonic()));
        opMode.telemetry.addData("LF Encoder", String.valueOf(robot.motorLF.getCurrentPosition()));
        opMode.telemetry.addData("LR Encoder", String.valueOf(robot.motorLR.getCurrentPosition()));
        opMode.telemetry.addData("RF Encoder", String.valueOf(robot.motorRF.getCurrentPosition()));
        opMode.telemetry.addData("RR Encoder", String.valueOf(robot.motorRR.getCurrentPosition()));
        opMode.telemetry.addData("LF", String.valueOf(LF));
        opMode.telemetry.addData("RF", String.valueOf(RF));
        opMode.telemetry.addData("LR", String.valueOf(LR));
        opMode.telemetry.addData("RR", String.valueOf(RR));
        opMode.telemetry.update();
    }

    /**
     * Log data to the file on the phone.
     */
    private void logData() {

        Dl.addField(String.valueOf(runtime.time()));

        Dl.addField(String.valueOf(LF));
        Dl.addField(String.valueOf(RF));
        Dl.addField(String.valueOf(LR));
        Dl.addField(String.valueOf(RR));
        Dl.newLine();
    }
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

}
