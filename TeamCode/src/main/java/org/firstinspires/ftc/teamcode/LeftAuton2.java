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

@Config

@Autonomous(name="LeftAuton-0+4", preselectTeleOp = "WaldonTeleOp")

public class LeftAuton2 extends LinearOpMode {
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

        RevBlinkinLedDriver LED = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        RevBlinkinLedDriver.BlinkinPattern pattern;
        pattern = RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_FOREST_PALETTE;
        LED.setPattern(pattern);


        Action deliverSample0 = drive.actionBuilder(beginPose)
                .splineToLinearHeading(new Pose2d(-58.25,-58.25,Math.toRadians(225)),Math.toRadians(180))
                .build();

        Action pickupSample1 = drive.actionBuilder(new Pose2d(-58.25, -58.25, Math.toRadians(225)))
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-39,-32,Math.toRadians(0)),Math.toRadians(180))
                .build();

        Action deliverSample1 = drive.actionBuilder(new Pose2d(-39, -32,Math.toRadians(0)))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-58.25,-58.25,Math.toRadians(225)),Math.toRadians(180))
                .build();

        Action pickupSample2 = drive.actionBuilder(new Pose2d(-58.25,-58.25,Math.toRadians(225)))
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-47,-32,Math.toRadians(0)),Math.toRadians(180))
                .build();
//
        Action deliverSample2 = drive.actionBuilder(new Pose2d(-47,-32,Math.toRadians(0)))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-58.25,-58.25,Math.toRadians(225)),Math.toRadians(180))
                .build();

        Action pickupSample3 = drive.actionBuilder(new Pose2d(-58.25,-58.25,Math.toRadians(225)))
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-55,-32,Math.toRadians(0)),Math.toRadians(180))
                .build();

        Action deliverSample3 = drive.actionBuilder(new Pose2d(-55,-32,Math.toRadians(0)))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-58.25,-58.25,Math.toRadians(225)),Math.toRadians(180))
                .build();


        waitForStart();
        Actions.runBlocking(
                new SequentialAction(
                        deliverSample0,
                        DiveActions.Lift.liftToHighBasket(),
                        DiveActions.Intake.Stop(),
                        DiveActions.SampleDelivery.load(),
                        new SleepAction(1),
                        DiveActions.SampleDelivery.dump(),
                        new SleepAction(0.5),
                        DiveActions.Lift.liftFullDown(),
                        //////////////////////////////////////////////////////////////////
                        // sample 1
                        //////////////////////////////////////////////////////////////////
                        DiveActions.Intake.wristdown(),
                        new ParallelAction(
                                DiveActions.Intake.wheelOn(),
                                pickupSample1
                        ),
                        DiveActions.Intake.wristUp(),
                        DiveActions.Intake.Stop(),
                        deliverSample1,
                        DiveActions.Intake.RevWheel(),
                        new SleepAction(0.5),
                        DiveActions.Lift.liftToHighBasket(),
                        DiveActions.Intake.Stop(),
                        DiveActions.SampleDelivery.load(),
                        new SleepAction(1),
                        DiveActions.SampleDelivery.dump(),
                        new SleepAction(0.5),
                        DiveActions.Lift.liftFullDown(),
                        //////////////////////////////////////////////////////////////////
                        // sample 2
                        //////////////////////////////////////////////////////////////////
                        DiveActions.Intake.wristdown(),
                        new ParallelAction(
                                DiveActions.Intake.wheelOn(),
                                pickupSample2
                        ),
                        DiveActions.Intake.wristUp(),
                        DiveActions.Intake.Stop(),
                        deliverSample2,
                        DiveActions.Intake.RevWheel(),
                        new SleepAction(0.5),
                        DiveActions.Lift.liftToHighBasket(),
                        DiveActions.Intake.Stop(),
                        DiveActions.SampleDelivery.load(),
                        new SleepAction(1),
                        DiveActions.SampleDelivery.dump(),
                        new SleepAction(0.5),
                        DiveActions.Lift.liftFullDown(),
                        //////////////////////////////////////////////////////////////////
                        // sample 3
                        //////////////////////////////////////////////////////////////////
                        DiveActions.Intake.wristdown(),
                        new ParallelAction(
                                DiveActions.Intake.wheelOn(),
                                pickupSample3
                        ),
                        DiveActions.Intake.wristUp(),
                        DiveActions.Intake.Stop(),
                        deliverSample3,
                        DiveActions.Intake.RevWheel(),
                        new SleepAction(0.5),
                        DiveActions.Lift.liftToHighBasket(),
                        DiveActions.Intake.Stop(),
                        DiveActions.SampleDelivery.load(),
                        new SleepAction(1),
                        DiveActions.SampleDelivery.dump(),
                        new SleepAction(0.5),
                        DiveActions.Lift.liftFullDown(),
                        new SleepAction(2) // for some reason, we need this extra sleep to ensure that the bucket goes all the way down at the end.
                )
        );
        //Actions.runBlocking(new SequentialAction(DiveActions.Intake.extendArm()));
    }

}