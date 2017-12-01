package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareProfiles.HardwareTestPlatform;
import org.firstinspires.ftc.teamcode.Libs.DriveMecanum;
import org.firstinspires.ftc.teamcode.Libs.DataLogger;

/**
 * Created by MyPC on 11/8/2017.
 */
@Autonomous(name="ColorSensor", group="COMP")

public class colorsensor extends LinearOpMode  {

    private final static HardwareTestPlatform robot = new HardwareTestPlatform();
    private LinearOpMode opMode = this;
    private State state = State.DRIVE;
    private double colorRight = 0;
    private double colorLeft = 0;
    private double colorLeftBlue = 0;
    private double colorLeftRed = 0;
    private DataLogger Dl;


    public void runOpMode() {
        robot.init(hardwareMap);
        DriveMecanum drive = new DriveMecanum(robot, opMode, Dl);

        robot.servoRight.setPosition(0);                                                        //Make sure color sensor arms are up,  Servo
        robot.servoLeft.setPosition (0);

        while (opModeIsActive()) {

            robot.servoRight.setPosition(0.25);                                                        //Make sure color sensor arms are up,  Servo
            robot.servoLeft.setPosition (0.25);



            switch (state) {
                case DRIVE:

                    robot.blockHolder.setPower(1);

                    sleep(500);

                    robot.blockHolder.setPower(0);



                if (robot.colorSensorLeft.blue() > robot.colorSensorLeft.red()) {  //Blue is back

                    drive.translateTime(.4, .5, 0);
                }

                break;


                case COLOR_SENSOR:
                        colorRight = robot.colorSensorRight.argb();
                        colorLeft = robot.colorSensorLeft.argb();
                        telemetry.addData("Right RGB", String.valueOf(colorRight));
                        telemetry.addData("Right Blue", String.valueOf(robot.colorSensorRight.blue()));
                        telemetry.addData("Right Red", String.valueOf(robot.colorSensorRight.red()));
                        telemetry.addData("Left RGB", String.valueOf(colorLeft));
                        telemetry.addData("Left Blue", String.valueOf(robot.colorSensorLeft.blue()));
                        telemetry.addData("Left Red", String.valueOf(robot.colorSensorLeft.red()));
                        telemetry.update();



            }
        }


    }
    private void telemetry() {
        opMode.telemetry.addData("Right RGB", String.valueOf(colorRight));
        opMode.telemetry.addData("Right Blue", String.valueOf(robot.colorSensorRight.blue()));
        opMode.telemetry.addData("Right Red", String.valueOf(robot.colorSensorRight.red()));
        opMode.telemetry.addData("Left RGB", String.valueOf(colorLeft));
        opMode.telemetry.addData("Left Blue", String.valueOf(robot.colorSensorLeft.blue()));
        opMode.telemetry.addData("Left Red", String.valueOf(robot.colorSensorLeft.red()));
        opMode.telemetry.update();

    }

    enum State {
        HALT, DRIVE, TAIL, COLOR_SENSOR, RANGE
    }
}
