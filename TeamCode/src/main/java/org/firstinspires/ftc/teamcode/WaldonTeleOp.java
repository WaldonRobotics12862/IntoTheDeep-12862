package org.firstinspires.ftc.teamcode;
//package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp(name="WaldonTeleOp")

public class WaldonTeleOp extends LinearOpMode {
    @Override
        public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotorEx frontLeftMotor = hardwareMap.get(DcMotorEx.class,"leftFront");
        DcMotorEx backLeftMotor = hardwareMap.get(DcMotorEx.class,"leftBack");
        DcMotorEx frontRightMotor = hardwareMap.get(DcMotorEx.class,"rightBack");
        DcMotorEx backRightMotor = hardwareMap.get(DcMotorEx.class,"rightFront");
        DcMotorEx ascendMotor = hardwareMap.get(DcMotorEx.class, "ascend");
        IMU imu = hardwareMap.get(IMU.class, "imu");

        DcMotorEx lift = hardwareMap.get(DcMotorEx.class, "leftLift");

        DataLogger dataLog = new DataLogger("TeleOp_log");
        dataLog.addField("Color Sensor");
        dataLog.addField("Lift Height");
        dataLog.newLine();

        double lastPressedX = 0;
        double lastPressedY = 0;
        double lastPressedA = 0;
        double lastPressedB = 0;

        boolean intakeRunning = false;
        boolean ExtendForward = false;
        boolean LiftGoing = false;
        boolean BucketUp = false;
        boolean wristUp = false;
        boolean wristdown = false;

        DiveActions.SpecimenDelivery SpecimenDelivery = new DiveActions.SpecimenDelivery(hardwareMap);
        DiveActions.Intake Intake = new DiveActions.Intake(hardwareMap);
        DiveActions.SampleDelivery SampleDelivery = new DiveActions.SampleDelivery(hardwareMap);
        DiveActions.Lift Lift = new DiveActions.Lift(hardwareMap);

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorEx.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorEx.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotorEx.Direction.FORWARD);
        ascendMotor.setDirection(DcMotorEx.Direction.FORWARD);

        ascendMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            dataLog.addField("Color Sensor");
            dataLog.addField(lift.getCurrentPosition());
            dataLog.newLine();

            if (gamepad1.a) {
                imu.resetYaw();
            }

            if (gamepad2.right_bumper){
                Actions.runBlocking(new SequentialAction(DiveActions.SpecimenDelivery.open()));
            }
            if (gamepad2.left_bumper){
                Actions.runBlocking(new SequentialAction(DiveActions.SpecimenDelivery.close()));
            }
            if (gamepad2.y && !ExtendForward && System.currentTimeMillis() - lastPressedY > 250){
                Actions.runBlocking(new SequentialAction(DiveActions.Intake.ExtendArm()));
                lastPressedY = System.currentTimeMillis();
                ExtendForward = true;
            }
            if (gamepad2.y && ExtendForward && System.currentTimeMillis() - lastPressedY > 250){
                Actions.runBlocking(new SequentialAction(DiveActions.Intake.RetractArm()));
                lastPressedY = System.currentTimeMillis();
                ExtendForward = false;
            }

            if(gamepad2.a && !intakeRunning && System.currentTimeMillis() - lastPressedA > 500) {
                Actions.runBlocking(new SequentialAction(DiveActions.Intake.WheelOn()));
                lastPressedA = System.currentTimeMillis();
                intakeRunning = true;
            }
            if (gamepad2.a && intakeRunning && System.currentTimeMillis() - lastPressedA > 500){
                Actions.runBlocking(new SequentialAction(DiveActions.Intake.Stop()));
                lastPressedA = System.currentTimeMillis();
                intakeRunning = false;
            }

            if(gamepad2.dpad_up)  {
                Actions.runBlocking(new SequentialAction(DiveActions.Lift.LiftUp()));
            }
            if(gamepad2.dpad_down ) {
                Actions.runBlocking(new SequentialAction(DiveActions.Lift.LiftDown()));
            }
            if(gamepad2.x && !BucketUp && System.currentTimeMillis() - lastPressedA > 500){
                Actions.runBlocking(new SequentialAction(DiveActions.SampleDelivery.load()));
                lastPressedX = System.currentTimeMillis();
                BucketUp = false;
            }
            if(gamepad2.x && BucketUp && System.currentTimeMillis() - lastPressedA > 500){
                Actions.runBlocking(new SequentialAction(DiveActions.SampleDelivery.dump()));
                lastPressedX = System.currentTimeMillis();
                BucketUp = true;
            }
              if(gamepad2.right_bumper){
               Actions.runBlocking((new SequentialAction(DiveActions.SpecimenDelivery.open())));
            }
            if(gamepad2.left_bumper){
                Actions.runBlocking((new SequentialAction(DiveActions.SpecimenDelivery.close())));
            }

            //Ascend Motor Control
            ascendMotor.setPower(gamepad2.right_trigger - gamepad2.left_trigger);

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
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

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
        }
        dataLog.closeDataLogger();
    }
}
