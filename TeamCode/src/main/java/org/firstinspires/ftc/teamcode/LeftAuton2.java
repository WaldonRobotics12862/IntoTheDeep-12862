package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Config

@Autonomous(name="LeftAuton-0+4", preselectTeleOp = "WaldonTeleOp")

public class LeftAuton2 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-13, -61, Math.toRadians(90));
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


        Action deliverSample0 = drive.actionBuilder(beginPose)
                .splineToLinearHeading(new Pose2d(-54,-58,Math.toRadians(225)),Math.toRadians(180))
                .build();

        Action pickupSample1 = drive.actionBuilder(new Pose2d(-54, -58, Math.toRadians(225)))
                .setTangent(Math.toRadians(-90))
                .splineToSplineHeading((new Pose2d(-58,-43,Math.toRadians(240))),Math.toRadians(90))
                // strafe straight in X
                .setTangent(120)
                .splineToConstantHeading(new Vector2d(-55.5,-40),Math.toRadians(240))
                .build();

        Action deliverSample1 = drive.actionBuilder(new Pose2d(-55.5, -40,Math.toRadians(240)))
                .splineToLinearHeading(new Pose2d(-54,-58,Math.toRadians(225)),Math.toRadians(180))
                .build();

        Action pickupSample2 = drive.actionBuilder(new Pose2d(-54,-58,Math.toRadians(225)))
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-46,-40,Math.toRadians(-45)),Math.toRadians(180))
                .build();

        Action deliverSample2 = drive.actionBuilder(new Pose2d(-46,-40,Math.toRadians(-45)))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-54,-58,Math.toRadians(225)),Math.toRadians(180))
                .build();

        Action pickupSample3 = drive.actionBuilder(new Pose2d(-54,-58,Math.toRadians(225)))
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-55.5,-43,Math.toRadians(-45)),Math.toRadians(180))
                .build();

        Action deliverSample3 = drive.actionBuilder(new Pose2d(-55.5,-43,Math.toRadians(-45)))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-54,-58,Math.toRadians(225)),Math.toRadians(180))
                .build();

        waitForStart();
        Actions.runBlocking(
                new SequentialAction(
                        deliverSample0,
                        DiveActions.Lift.liftToHighBasket(),
                        DiveActions.Intake.Stop(),
                        DiveActions.SampleDelivery.load(),
                        new SleepAction(0.5),
                        DiveActions.Lift.liftFullDown(),
                        new ParallelAction(
                                DiveActions.SampleDelivery.dump(),
                                DiveActions.Lift.liftFullDown()
                        ),
//                        DiveActions.SampleDelivery.dump(),
//                        new SleepAction(0.2),
//                        DiveActions.Lift.liftFullDown(),
//                        new SleepAction(0.2),
                        //////////////////////////////////////////////////////////////////
                        // sample 1
                        //////////////////////////////////////////////////////////////////
                        DiveActions.Intake.wristdown(),
                        new SleepAction (0.1),
                        DiveActions.Intake.wristdown(),
                        new ParallelAction(
//                                DiveActions.Lift.liftFullDown(),
                                DiveActions.Intake.wristdown(),
                                DiveActions.Intake.wheelOn(),
                                pickupSample1
                        ),
                        new SleepAction(0.5), // pause for just a little to make sure we pull in
                        DiveActions.Intake.wristUp(),
                        DiveActions.Intake.Stop(),
                        deliverSample1,
                        DiveActions.Intake.RevWheel(),
                        new SleepAction(0.5),
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
                        new SleepAction(0.5),
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
                        new SleepAction(0.5),
                        DiveActions.Lift.liftToHighBasket(),
                        DiveActions.Intake.Stop(),
                        DiveActions.SampleDelivery.load(),
                        new SleepAction(0.5),
                        DiveActions.SampleDelivery.dump(),
                        new SleepAction(0.1),
                        DiveActions.Lift.liftFullDown(),
                        new SleepAction(0.5), // for some reason, we need this extra sleep to ensure that the bucket goes all the way down at the end.
                        new SleepAction(0.5)
                )
        );
        //Actions.runBlocking(new SequentialAction(DiveActions.Intake.extendArm()));
    }

}