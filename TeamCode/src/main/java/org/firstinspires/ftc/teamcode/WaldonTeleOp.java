package org.firstinspires.ftc.teamcode;
//package org.firstinspires.ftc.teamcode.tuning;

import android.graphics.Color;

import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp(name="WaldonTeleOp")

public class WaldonTeleOp extends LinearOpMode {


    double lastPressedX = 0;
    double lastPressedY = 0;
    double lastPressedA = 0;
    double lastPressedB = 0;

    boolean ExtendForward = false;
    boolean intakeRunning = false;
    boolean LiftGoing = false;
    boolean BucketUp = false;
    boolean wristUp = false;
    boolean wristdown = false;


    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx frontLeftMotor = hardwareMap.get(DcMotorEx.class,"leftFront");
        DcMotorEx backLeftMotor = hardwareMap.get(DcMotorEx.class,"leftBack");
        DcMotorEx frontRightMotor = hardwareMap.get(DcMotorEx.class,"rightBack");
        DcMotorEx backRightMotor = hardwareMap.get(DcMotorEx.class,"rightFront");
        DcMotorEx ascendMotor = hardwareMap.get(DcMotorEx.class, "ascend");
        IMU imu = hardwareMap.get(IMU.class, "imu");
        RevColorSensorV3 color = hardwareMap.get(RevColorSensorV3.class, "sampleColor");
        RevBlinkinLedDriver LED = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        DcMotorEx lift = hardwareMap.get(DcMotorEx.class, "leftLift");

        // Adjust the orientation parameters to match your robot
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));

        imu.initialize(parameters);

        RevBlinkinLedDriver.BlinkinPattern pattern;
        pattern = RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_FOREST_PALETTE;
        LED.setPattern(pattern);

        frontLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorEx.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorEx.Direction.FORWARD);

        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ascendMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        DataLogger dataLog = new DataLogger("TeleOp_log");
        dataLog.addField("Color Sensor");
        dataLog.addField("Lift Height");
        dataLog.newLine();

        DiveActions.SpecimenDelivery SpecimenDelivery = new DiveActions.SpecimenDelivery(hardwareMap);
        DiveActions.Intake Intake = new DiveActions.Intake(hardwareMap);
        DiveActions.SampleDelivery SampleDelivery = new DiveActions.SampleDelivery(hardwareMap);
        DiveActions.Lift Lift = new DiveActions.Lift(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            //Drive();
            RunIntake();
            RunDelivery();

            //Get Color Sensor Data
            int myColor = color.getNormalizedColors().toColor();
            double hue = JavaUtil.rgbToHue(Color.red(myColor), Color.green(myColor), Color.blue(myColor));

            telemetry.addData("hue: ",hue);
            telemetry.update();

            dataLog.addField(hue);
            dataLog.addField(lift.getCurrentPosition());
            dataLog.newLine();


            //Set LEDs
            if (hue > 210 && hue < 240) {
                pattern = RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE;
                LED.setPattern(pattern);
                telemetry.addData("pattern:", pattern );
            } else if (hue > 75 && hue < 90) {
                pattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
                LED.setPattern(pattern);
                telemetry.addData("pattern:", pattern );

            } else if (hue > 10 && hue < 30) {
            pattern = RevBlinkinLedDriver.BlinkinPattern.DARK_RED;
            LED.setPattern(pattern);
            telemetry.addData("pattern:", pattern );

            } else {
                pattern = RevBlinkinLedDriver.BlinkinPattern.LAWN_GREEN;
                LED.setPattern(pattern);
                telemetry.addData("pattern:", pattern);
            }


            // DRIVE IS HERE:
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            // Rotate the movement direction counter to the bot's rotation
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            if (gamepad1.a) {
                imu.resetYaw();
            }

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY + rotX - rx) / denominator;
            double backRightPower = (rotY - rotX - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
            //Ascend Motor Control
            ascendMotor.setPower(gamepad2.right_trigger - gamepad2.left_trigger);
        }
        dataLog.closeDataLogger();
    }

    private void RunIntake(){
        if (gamepad2.y && !ExtendForward && System.currentTimeMillis() - lastPressedY > 500){
            Actions.runBlocking(new SequentialAction(DiveActions.Intake.extendArm(telemetry)));
            lastPressedY = System.currentTimeMillis();
            ExtendForward = true;
        }
        if (gamepad2.y && ExtendForward && System.currentTimeMillis() - lastPressedY > 500){
            Actions.runBlocking(new SequentialAction(DiveActions.Intake.retractArm()));
            lastPressedY = System.currentTimeMillis();
            ExtendForward = false;
        }
        if(gamepad2.a && !intakeRunning && System.currentTimeMillis() - lastPressedA > 500) {
            Actions.runBlocking(new SequentialAction(DiveActions.Intake.wheelOn()));
            lastPressedA = System.currentTimeMillis();
            intakeRunning = true;
        }
        if (gamepad2.a && intakeRunning && System.currentTimeMillis() - lastPressedA > 500){
            Actions.runBlocking(new SequentialAction(DiveActions.Intake.Stop()));
            lastPressedA = System.currentTimeMillis();
            intakeRunning = false;
        }
        if(gamepad2.b && !intakeRunning && System.currentTimeMillis() - lastPressedB > 500) {
            Actions.runBlocking((new SequentialAction(DiveActions.Intake.RevWheel())));
            lastPressedB = System.currentTimeMillis();
            intakeRunning = true;
        }
        if(gamepad2.b && intakeRunning && System.currentTimeMillis() - lastPressedB > 500) {
            Actions.runBlocking((new SequentialAction(DiveActions.Intake.Stop())));
            lastPressedB = System.currentTimeMillis();
            intakeRunning = false;
        }
    }

    private void RunDelivery(){
        if(gamepad2.dpad_up)  {
            Actions.runBlocking(new SequentialAction(DiveActions.Lift.LiftUp()));
        }
        if(gamepad2.dpad_down ) {
            Actions.runBlocking(new SequentialAction(DiveActions.Lift.LiftDown()));
        }
        if(gamepad2.x && !BucketUp && System.currentTimeMillis() - lastPressedX > 500){
            Actions.runBlocking(new SequentialAction(DiveActions.SampleDelivery.load()));
            lastPressedX = System.currentTimeMillis();
            BucketUp = true;
        }
        if(gamepad2.x && BucketUp && System.currentTimeMillis() - lastPressedX > 500){
            Actions.runBlocking(new SequentialAction(DiveActions.SampleDelivery.dump()));
            lastPressedX = System.currentTimeMillis();
            BucketUp = false;
        }
        if(gamepad2.right_bumper){
            Actions.runBlocking((new SequentialAction(DiveActions.SpecimenDelivery.open())));
        }
        if(gamepad2.left_bumper){
            Actions.runBlocking((new SequentialAction(DiveActions.SpecimenDelivery.close())));
        }
    }
}
