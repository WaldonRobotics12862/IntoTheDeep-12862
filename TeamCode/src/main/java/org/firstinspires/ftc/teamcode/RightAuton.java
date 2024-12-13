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
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="RightAuton-2", preselectTeleOp = "WaldonTeleOp")
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

        Servo ExtendIntake = hardwareMap.get(Servo.class, "intakeExtend");
        ExtendIntake.setPosition(0);

        Servo WristServo = hardwareMap.get(Servo.class, "intakeWrist");
        WristServo.setPosition(Variables.wristUp);

        RevBlinkinLedDriver LED = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        RevBlinkinLedDriver.BlinkinPattern pattern;
        pattern = RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_FOREST_PALETTE;
        LED.setPattern(pattern);

        //RIGHT AUTON PATH
        //This delivers one, then goes and pushes the specimen over into the observations zone and
        // waits to pick up a second specimen so that the human player can create it.
        // This path is only used if our partner DOES load a specimen but this is NOT our preferred
        // path

        Action Deliver1 = drive.actionBuilder(new Pose2d(15, -63, Math.toRadians(90)))
                .lineToY(-32)
                .build();

        Action backup2 = drive.actionBuilder(new Pose2d(0, 0, Math.toRadians(90)))
                .lineToY(-2)
                .build();

        Action Pickup2 = drive.actionBuilder(new Pose2d(15, -33, Math.toRadians(90)))
                .lineToY(-38)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(18,-38, Math.toRadians(90)), 0) // this should strafe right
                .splineTo(new Vector2d(33,-12), Math.toRadians(90))
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(60,-55, Math.toRadians(180)),Math.toRadians(45)) // push the sample over to the observation zone
                .setTangent(0)
                .lineToX(55)
                .splineToLinearHeading(new Pose2d(40, -30, Math.toRadians(-90)), 0)
                .build();

        Action Pickup22 = drive.actionBuilder(new Pose2d(40, -30, Math.toRadians(270)))
                .lineToY(-66)
                .build();

        Action Deliver2 = drive.actionBuilder(new Pose2d(40, -63, Math.toRadians(270)))
                .lineToY(-58)
                .turn(Math.toRadians(-90))
                .splineTo(new Vector2d(6, -28),Math.toRadians(90))
                .build();

        Action Park = drive.actionBuilder(new Pose2d(0, -33, Math.toRadians(90)))
                .lineToY(-38)
                .turn(Math.toRadians(-120)) // was 90
                .lineToX(48)
//                .splineTo(new Vector2d(40, -68), Math.toRadians(270))
                .build();


        waitForStart();
        Actions.runBlocking(
                new SequentialAction(
                        Deliver1,
                        DiveActions.Lift.liftToHeight(Variables.HighChamberDeliver),
                        new SleepAction(0.1),
                        backup2,
                        DiveActions.Lift.liftFullDown(),
                        Pickup2,
                        new SleepAction(4),// this is the pause to let someone build the specimen
                        Pickup22,
                        DiveActions.SpecimenDelivery.close(),
//                        DiveActions.Lift.liftToHeight(Variables.HighChamberDeliver),
//                        DiveActions.Lift.liftToHighChamber(),
//                        new DiveActions.Lift.AutonDown(),
                        new ParallelAction(
                                DiveActions.Lift.liftToHighChamber(),
                                Deliver2
                        ),
                        DiveActions.Lift.liftToHeight(Variables.HighChamberDeliver),
                        new SleepAction(0.2 ),
                        DiveActions.SpecimenDelivery.open(),
                        DiveActions.Lift.liftFullDown(),
                        Park
                )
        );
    }
}