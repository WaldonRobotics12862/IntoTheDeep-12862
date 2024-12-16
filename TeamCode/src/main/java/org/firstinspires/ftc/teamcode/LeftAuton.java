package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Config

@Autonomous(name="LeftAuton-1+3", preselectTeleOp = "WaldonTeleOp")

public class LeftAuton extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-15, -63, Math.toRadians(90));
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, beginPose);
        DiveActions.Lift Lift = new DiveActions.Lift(hardwareMap);
        DiveActions.SpecimenDelivery SpecimenDelivery = new DiveActions.SpecimenDelivery(hardwareMap);
        DiveActions.SampleDelivery SampleDelivery = new DiveActions.SampleDelivery(hardwareMap);
        //DiveActions.LED LED = new DiveActions.LED(hardwareMap);
        DiveActions.Ascend Ascend = new DiveActions.Ascend(hardwareMap);
        DiveActions.Intake Intake = new DiveActions.Intake(hardwareMap);

        DcMotorEx liftLeft = hardwareMap.get(DcMotorEx.class, "leftLift");
        liftLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        liftLeft.setDirection(DcMotorEx.Direction.FORWARD);
        liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        DcMotorEx liftRight = hardwareMap.get(DcMotorEx.class, "rightLift");
        liftRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        liftRight.setDirection(DcMotorEx.Direction.REVERSE);
        liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        Servo ExtendIntake = hardwareMap.get(Servo.class, "intakeExtend");
        ExtendIntake.setPosition(0);

        Servo WristServo = hardwareMap.get(Servo.class, "intakeWrist");
        WristServo.setPosition(Variables.wristUp);

        RevBlinkinLedDriver LED = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        RevBlinkinLedDriver.BlinkinPattern pattern;
        pattern = RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_FOREST_PALETTE;
        LED.setPattern(pattern);

        Action DeliverSpecimen = drive.actionBuilder(beginPose)
                .lineToY(-32)
                .build();

        Action backup = drive.actionBuilder(new Pose2d(-15, -32, Math.toRadians(90)))
                .lineToY(-38)
                .build();

        Action pickupSample1 = drive.actionBuilder(new Pose2d(-15, -38, Math.toRadians(90)))
                .setTangent(Math.toRadians(-90))
                .splineToSplineHeading((new Pose2d(-24,-48,Math.toRadians(0))),Math.toRadians(245))
                .splineToLinearHeading(new Pose2d(-35,-31,Math.toRadians(-45)),Math.toRadians(180))
                .build();

        Action deliverSample1 = drive.actionBuilder(new Pose2d(-35, -31,Math.toRadians(-45)))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-57,-57,Math.toRadians(225)),Math.toRadians(180))
                .build();

        Action pickupSample2 = drive.actionBuilder(new Pose2d(-57,-57,Math.toRadians(225)))
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-46,-42,Math.toRadians(-45)),Math.toRadians(180))
                .build();

        Action deliverSample2 = drive.actionBuilder(new Pose2d(-46,-42,Math.toRadians(-45)))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-57,-57,Math.toRadians(225)),Math.toRadians(180))
                .build();

        Action pickupSample3 = drive.actionBuilder(new Pose2d(-57,-57,Math.toRadians(225)))
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-55,-42,Math.toRadians(-35)),Math.toRadians(180))
                .build();

        Action deliverSample3 = drive.actionBuilder(new Pose2d(-55,-42,Math.toRadians(-35)))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-57,-57,Math.toRadians(225)),Math.toRadians(180))
                .build();


        waitForStart();
        Actions.runBlocking(
                new SequentialAction(
                        DeliverSpecimen,
                        DiveActions.Lift.liftToHeight(-950),
                        new SleepAction(0.1),
                        new ParallelAction(
                                backup,
                                DiveActions.Lift.liftFullDown()
                        ),
                        DiveActions.Lift.liftFullDown(),
                        DiveActions.SampleDelivery.dump(),
                        //////////////////////////////////////////////////////////////////
                        // sample 1
                        //////////////////////////////////////////////////////////////////
                        DiveActions.Intake.wristdown(),
                        new ParallelAction(
                                DiveActions.Intake.wristdown(),
                                DiveActions.Intake.wheelOn(),
                                pickupSample1
                        ),
                        new SleepAction(0.5), // pause for just a little to make sure we pull in
                        DiveActions.Intake.wristUp(),
                        DiveActions.Intake.Stop(),
                        deliverSample1,
                        DiveActions.Intake.RevWheel(),
                        new SleepAction(0.4),
                        DiveActions.Lift.liftToHighBasket(),
                        DiveActions.Intake.Stop(),
                        DiveActions.SampleDelivery.load(),
                        new SleepAction(0.5),
                        DiveActions.SampleDelivery.dump(),
                        new SleepAction(0.1),
                        DiveActions.Lift.liftFullDown(),
//                        //////////////////////////////////////////////////////////////////
//                        // sample 2
//                        //////////////////////////////////////////////////////////////////
                        DiveActions.Intake.wristdown(),
                        new ParallelAction(
                                DiveActions.Intake.wheelOn(),
                                pickupSample2
                        ),
                        new SleepAction(0.5),
                        DiveActions.Intake.wristUp(),
                        DiveActions.Intake.Stop(),
                        deliverSample2,
                        DiveActions.Intake.RevWheel(),
                        new SleepAction(0.4),
                        DiveActions.Lift.liftToHighBasket(),
                        DiveActions.Intake.Stop(),
                        DiveActions.SampleDelivery.load(),
                        new SleepAction(0.5),
                        DiveActions.SampleDelivery.dump(),
                        new SleepAction(0.1),
                        DiveActions.Lift.liftFullDown(),
                        //////////////////////////////////////////////////////////////////
                        // sample 3
                        //////////////////////////////////////////////////////////////////
                        DiveActions.Intake.wristdown(),
                        new ParallelAction(
                                DiveActions.Intake.wheelOn(),
                                pickupSample3
                        ),
                        new SleepAction (0.5),
                        DiveActions.Intake.wristUp(),
                        DiveActions.Intake.Stop(),
                        deliverSample3,
                        DiveActions.Intake.RevWheel(),
                        new SleepAction(0.4),
                        DiveActions.Lift.liftToHighBasket(),
                        DiveActions.Intake.Stop(),
                        DiveActions.SampleDelivery.load(),
                        new SleepAction(0.5),
                        DiveActions.SampleDelivery.dump(),
                        new SleepAction(0.1),
                        DiveActions.Lift.liftFullDown(),
                        new SleepAction(2), // for some reason, we need this extra sleep to ensure that the bucket goes all the way down at the end.
                        new SleepAction(1)
                )

        );
        //Actions.runBlocking(new SequentialAction(DiveActions.Intake.extendArm()));
    }

}