package org.firstinspires.ftc.teamcode;
//package org.firstinspires.ftc.teamcode.tuning;

import android.graphics.Color;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


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
    public boolean bucketUp = false;
    boolean wristUp = false;
    boolean wristdown = false;
    boolean startedAscend = false;

    double slow_mode = 1;

    double extendPos = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx frontLeftMotor = hardwareMap.get(DcMotorEx.class,"leftFront");
        DcMotorEx backLeftMotor = hardwareMap.get(DcMotorEx.class,"leftBack");
        DcMotorEx frontRightMotor = hardwareMap.get(DcMotorEx.class,"rightBack");
        DcMotorEx backRightMotor = hardwareMap.get(DcMotorEx.class,"rightFront");
        DcMotorEx ascendMotor = hardwareMap.get(DcMotorEx.class, "ascend");
//        Servo ExtendIntake = hardwareMap.get(Servo.class, "intakeExtend");
        IMU imu = hardwareMap.get(IMU.class, "imu");
        Servo ExtendIntake = hardwareMap.get(Servo.class, "intakeExtend");
        RevColorSensorV3 color = hardwareMap.get(RevColorSensorV3.class, "sampleColor");
        RevBlinkinLedDriver LED = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        DcMotorEx liftR = hardwareMap.get(DcMotorEx.class, "rightLift");
        DcMotorEx liftL = hardwareMap.get(DcMotorEx.class, "leftLift");

        liftR.setDirection(DcMotorEx.Direction.REVERSE);

        liftL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        liftR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        liftL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        liftR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        ascendMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        ascendMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Adjust the orientation parameters to match your robot
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));

        imu.initialize(parameters);

        RevBlinkinLedDriver.BlinkinPattern pattern;
        pattern = RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_FOREST_PALETTE;
        LED.setPattern(pattern);
        //LED.resetDeviceConfigurationForOpMode();

        frontLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorEx.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorEx.Direction.REVERSE);

        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

            Drive(frontLeftMotor,backLeftMotor, frontRightMotor, backRightMotor, imu);
            RunIntake(ExtendIntake);
            RunDelivery(liftL, liftR);

            //Get Color Sensor Data
            int myColor = color.getNormalizedColors().toColor();
            double hue = JavaUtil.rgbToHue(Color.red(myColor), Color.green(myColor), Color.blue(myColor));


            dataLog.addField(hue);
            dataLog.addField(liftL.getCurrentPosition());
            dataLog.addField(liftR.getCurrentPosition());
            dataLog.newLine();

            //Set LEDs
            if (hue > 181 && hue < 240) {
                pattern = RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE;
                LED.setPattern(pattern);
            } else if (hue > 75 && hue < 100) {
                pattern = RevBlinkinLedDriver.BlinkinPattern.GOLD;
                LED.setPattern(pattern);
            } else if (hue > 10 && hue < 50) {
                pattern = RevBlinkinLedDriver.BlinkinPattern.DARK_RED;
                LED.setPattern(pattern);
            } else {
                pattern = RevBlinkinLedDriver.BlinkinPattern.DARK_GREEN;
                LED.setPattern(pattern);
            }



            // DRIVE IS HERE:
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            // Rotate the movement direction counter to the bot's rotation
            if(gamepad1.right_bumper) {
                slow_mode = .5;
            } else {
                slow_mode = 1;
            }
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
            double frontLeftPower = ((rotY + rotX + rx) / denominator) * slow_mode;
            double backLeftPower = ((rotY - rotX + rx) / denominator) * slow_mode;
            double frontRightPower = ((rotY + rotX - rx) / denominator) * slow_mode;
            double backRightPower = ((rotY - rotX - rx) / denominator) * slow_mode;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);


            //Ascend Motor Control
            if(!startedAscend){
                ascendMotor.setPower(gamepad2.right_trigger - gamepad2.left_trigger);
            }

            if(gamepad2.guide){
                if(startedAscend){
                    Actions.runBlocking(
                            new ParallelAction(
                                DiveActions.Lift.liftToHighBasket(),
                                DiveActions.SampleDelivery.load()
//                                DiveActions.Ascend.pullUp())
                            )
                    );
                    ascendMotor.setTargetPosition(200);
                    ascendMotor.setPower(1);
                    ascendMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    Actions.runBlocking(
                        new SequentialAction(
                            new SleepAction(2),
                            DiveActions.Lift.threeAscend()
                        )
                    );
                }
                else{
                    //ascendMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    startedAscend = true;
                }
                telemetry.addLine("I pressed guide");
                telemetry.update();
//                DiveActions.Ascend.pullUp();
//                ascendMotor.setTargetPosition(2000);
//                ascendMotor.setPower(1);
//                ascendMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            }

            if(gamepad1.guide){
//                if(startedAscend){
                    ascendMotor.setTargetPosition(150);
                    sleep(500);
                    ascendMotor.setTargetPosition(500);
                    Actions.runBlocking(
                            new SequentialAction(
                                    DiveActions.Lift.threeAscend2()
                            )
                    );
//                }
            }
        }



        dataLog.closeDataLogger();
    }

    private void RunIntake(Servo ExtendIntake){
        extendPos = gamepad2.left_stick_x;
        ExtendIntake.setPosition(extendPos);
        if(gamepad2.y && !ExtendForward && System.currentTimeMillis() - lastPressedY > 500){
            Actions.runBlocking(new SequentialAction(DiveActions.Intake.wristdown()));
//            Actions.runBlocking(new SequentialAction(DiveActions.Intake.extendArm(telemetry)));
            lastPressedY = System.currentTimeMillis();
            ExtendForward = true;
        }
        if(gamepad2.y && ExtendForward && System.currentTimeMillis() - lastPressedY > 500){
            Actions.runBlocking(new SequentialAction(DiveActions.Intake.wristUp()));
//            Actions.runBlocking(new SequentialAction(DiveActions.Intake.retractArm()));
            lastPressedY = System.currentTimeMillis();
            ExtendForward = false;
        }
        if(gamepad2.a && !intakeRunning && System.currentTimeMillis() - lastPressedA > 500) {
            Actions.runBlocking(new SequentialAction(DiveActions.Intake.wheelOn0()));
            lastPressedA = System.currentTimeMillis();
            intakeRunning = true;
        }
        if(gamepad2.a && intakeRunning && System.currentTimeMillis() - lastPressedA > 500){
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

    private void RunDelivery(DcMotorEx liftL, DcMotorEx liftR){

        if(gamepad2.dpad_up)  {
            Actions.runBlocking(new SequentialAction(DiveActions.Lift.liftToHeight2(Variables.HighBasket)));
            //Actions.runBlocking(new SequentialAction(DiveActions.Lift.liftToHighBasket()));
        }
        if(gamepad2.dpad_down ) {
            Actions.runBlocking(new SequentialAction(DiveActions.Lift.liftFullDown()));
        }
        if(gamepad2.dpad_left){
            Actions.runBlocking(new SequentialAction(DiveActions.Lift.liftToHeight2(Variables.HighChamber)));
            //Actions.runBlocking(new SequentialAction(DiveActions.Lift.liftToHighChamber()));
        }
        if(gamepad2.dpad_right){
            Actions.runBlocking(
                    new SequentialAction(
                            DiveActions.Lift.deliverHighChamber(),
                            DiveActions.SpecimenDelivery.open(),
                            new SleepAction(0.5),
                            DiveActions.Lift.liftFullDown()
                    )
            );
            //Actions.runBlocking(new SequentialAction(DiveActions.Lift.deliverHighChamber()));
        }

        if(gamepad2.start){
            Actions.runBlocking(new SequentialAction(DiveActions.Lift.liftToHeight(250)));
        }

        if(gamepad2.back){
            Actions.runBlocking(new SequentialAction(DiveActions.Lift.resetDown(System.currentTimeMillis())));
        }





        if(gamepad2.x && !bucketUp && System.currentTimeMillis() - lastPressedX > 500){
            Actions.runBlocking(new SequentialAction(DiveActions.SampleDelivery.load()));
            lastPressedX = System.currentTimeMillis();
            bucketUp = true;
        }
        if(gamepad2.x && bucketUp && System.currentTimeMillis() - lastPressedX > 500){
            Actions.runBlocking(new SequentialAction(DiveActions.SampleDelivery.dump()));
            lastPressedX = System.currentTimeMillis();
            bucketUp = false;
        }
        if(gamepad2.right_bumper){
            Actions.runBlocking((new SequentialAction(DiveActions.SpecimenDelivery.open())));
        }
        if(gamepad2.left_bumper){
            Actions.runBlocking((new SequentialAction(DiveActions.SpecimenDelivery.close())));
        }
    }

    private void Drive(DcMotorEx frontLeftMotor, DcMotorEx backLeftMotor, DcMotorEx frontRightMotor, DcMotorEx backRightMotor, IMU imu){
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        // Rotate the movement direction counter to the bot's rotation
        if(gamepad1.right_bumper) {
            slow_mode = .5;
        } else {
            slow_mode = 1;
        }
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
        double frontLeftPower = ((rotY + rotX + rx) / denominator) * slow_mode;
        double backLeftPower = ((rotY - rotX + rx) / denominator) * slow_mode;
        double frontRightPower = ((rotY + rotX - rx) / denominator) * slow_mode;
        double backRightPower = ((rotY - rotX - rx) / denominator) * slow_mode;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }

}
