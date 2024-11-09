package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name="RightAuton", preselectTeleOp = "WaldonTeleOp")
public final class RightAuton extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Define all the locations that we need
        Pose2d beginPose = new Pose2d(12, -63, Math.toRadians(90));

        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, beginPose);
        DiveActions.Lift Lift = new DiveActions.Lift(hardwareMap);
        DiveActions.SpecimenDelivery SpecimenDelivery = new DiveActions.SpecimenDelivery(hardwareMap);
        DiveActions.SampleDelivery SampleDelivery = new DiveActions.SampleDelivery(hardwareMap);
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



        Pose2d SP1p = new Pose2d(5, -33, Math.toRadians(90));

        Pose2d SP2 = new Pose2d(0, -32, Math.toRadians(90));
        Pose2d SP2p = new Pose2d(0, -33, Math.toRadians(90));

        Pose2d SP3 = new Pose2d(-5, -32, Math.toRadians(90));
        Pose2d SP3p = new Pose2d(-5, -33, Math.toRadians(90));

        Pose2d pickup = new Pose2d(40, -65, Math.toRadians(270));
        Pose2d pickupP = new Pose2d(40, -63, Math.toRadians(270));

        Action Deliver1 = drive.actionBuilder(beginPose)
                .splineTo(new Vector2d(5, -32),Math.toRadians(90))
                .build();

        Action Pickup2 = drive.actionBuilder(SP1p)
                .lineToY(-38)
                .turn(Math.toRadians(-90))
                .splineTo(new Vector2d(40, -70), Math.toRadians(270))
                .build();

        Action Deliver2 = drive.actionBuilder(pickupP)
                .lineToY(-58)
                .turn(Math.toRadians(-90))
                .splineTo(new Vector2d(0, -28),Math.toRadians(90))
                .build();

        Action Pickup3 = drive.actionBuilder(SP2p)
                .lineToY(-38)
                .turn(Math.toRadians(-90))
                .splineTo(new Vector2d(40, -70), Math.toRadians(270)) // was 48, -65
                .build();

        Action Deliver3 = drive.actionBuilder(pickupP)
                .lineToY(-58)
                .turn(Math.toRadians(-90))
                .splineTo(new Vector2d(-3, -26), Math.toRadians(90)) // was -5, -32
                .build();

        Action Park = drive.actionBuilder(SP3p)
                .lineToY(-38)
                .turn(Math.toRadians(-120)) // was 90
                .lineToX(48)
//                .splineTo(new Vector2d(40, -68), Math.toRadians(270))
                .build();


        waitForStart();
        Actions.runBlocking(
                new SequentialAction(
                        DiveActions.Lift.liftToHighChamber(),
                        Deliver1,
                        DiveActions.Lift.liftToHeight(Variables.HighChamberDeliver),
                        DiveActions.SpecimenDelivery.open(),
                        new SleepAction(0.5 ),
                        DiveActions.Lift.autonDown(),
                        Pickup2,
                        DiveActions.SpecimenDelivery.close(),
                        DiveActions.Lift.liftToHighChamber(),
                        new ParallelAction(
                                Deliver2,
                                DiveActions.Lift.liftToHighChamber()
                        ),
                        DiveActions.Lift.liftToHeight(Variables.HighChamberDeliver),
                        new SleepAction(0.5 ),
                        DiveActions.SpecimenDelivery.open(),
                        DiveActions.Lift.autonDown(),
                        Pickup3,
                        DiveActions.SpecimenDelivery.close(),
                        DiveActions.Lift.liftToHighChamber(),
                        new ParallelAction(
                                DiveActions.Lift.liftToHighChamber(),
                                Deliver3
                        ),
                        DiveActions.Lift.liftToHeight(Variables.HighChamberDeliver),
                        new SleepAction(0.5 ),
                        DiveActions.SpecimenDelivery.open(),
                        DiveActions.Lift.autonDown(),
                        Park
                )
       );
    }
}