//Autonomous methods to be used in run programs

package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.ImageFormat;
import android.os.Handler;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.Util;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.android.util.Size;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureRequest;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSequenceId;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSession;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCharacteristics;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraException;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraFrame;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraManager;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.collections.EvictingBlockingQueue;
import org.firstinspires.ftc.robotcore.internal.network.CallbackLooper;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.robotcore.internal.system.ContinuationSynchronizer;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.Locale;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.TimeUnit;

//for reading gyro


public class AutonomousMethods extends LinearOpMode {


    private static final String TAG = "Webcam: ";
    private int mNumRings=0;

    /** How long we are to wait to be granted permission to use the camera before giving up. Here,
     * we wait indefinitely */
    private static final int secondsPermissionTimeout = Integer.MAX_VALUE;

    /** State regarding our interaction with the camera */
    public CameraManager cameraManager;
    public WebcamName cameraName;
    public Camera camera;
    public CameraCaptureSession cameraCaptureSession;

    /** The queue into which all frames from the camera are placed as they become available.
     * Frames which are not processed by the OpMode are automatically discarded. */
    public EvictingBlockingQueue<Bitmap> frameQueue;

    /** State regarding where and how to save frames when the 'A' button is pressed. */
    public int captureCounter = 0;
    public File captureDirectory = AppUtil.ROBOT_DATA_DIR;

    /** A utility object that indicates where the asynchronous callbacks from the camera
     * infrastructure are to run. In this OpMode, that's all hidden from you (but see {@link #startCamera}
     * if you're curious): no knowledge of multi-threading is needed here. */
    private Handler callbackHandler;

    /*
     * Some color constants
     */
    //static final Scalar BLUE = new Scalar(0, 0, 255);
    //static final Scalar GREEN = new Scalar(0, 255, 0);
    /*
     * The core values which define the location and size of the sample regions
     */
    //static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(181,98);
    //static final int REGION_WIDTH = 35;
    //static final int REGION_HEIGHT = 25;
    //final int FOUR_RING_THRESHOLD = 150;
    //final int ONE_RING_THRESHOLD = 135;
    //Point region1_pointA = new Point(
    //        REGION1_TOPLEFT_ANCHOR_POINT.x,
    //        REGION1_TOPLEFT_ANCHOR_POINT.y);
//    Point region1_pointB = new Point(
//            REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
//            REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
//    int avg1;
//
    public Bitmap bmp;
    //private volatile EasyOpenCVExample.SkystoneDeterminationPipeline.RingPosition position = EasyOpenCVExample.SkystoneDeterminationPipeline.RingPosition.FOUR;



    public Hardware robot = new Hardware(true);
    private double gearRatio = 2;
    private double wheelDiameter = 4;
    //private double countsPerRotation = 1115; //counts per 360 degrees
    private double encoderCounts = 383.6*4; //counts per one rotation of output shaft
    private double currentxPosition = 0;
    private double currentyPosition = 0;
    public ElapsedTime runtime = new ElapsedTime();

    public void initializeRobot() {
        robot.initializeHardware(hardwareMap);

        //initializing camera
        callbackHandler = CallbackLooper.getDefault().getHandler();

        cameraManager = ClassFactory.getInstance().getCameraManager();
        cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        initializeFrameQueue(2);
        AppUtil.getInstance().ensureDirectoryExists(captureDirectory);
        if (!OpenCVLoader.initDebug()) {
            error("Error: Cannot load OpenCV LIbrary");
        } else {
            telemetry.addData(">", "Loaded OpenCV");
            telemetry.update();
        }

        try {
            openCamera();
            if (camera == null) return;

            startCamera();
            if (cameraCaptureSession == null) return;

            telemetry.addData(">", "Press Play to start");
            telemetry.update();
            waitForStart();


            bmp = frameQueue.poll();

        }
        finally {
            closeCamera();
        }
    }


    //moving forward distance (inch) with power [0, 1]
    public void forward(double power, double distance) {
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double angle = getHeading();
        int counts = (int) ((distance / (wheelDiameter * Math.PI)) * (encoderCounts / gearRatio));
        while (counts >= robot.backLeftMotor.getCurrentPosition()) {
            setAllMotorsTo(power);
        }

        //setting all motor powers to 0 (stopping)
        setAllMotorsTo(0);
        stopAndResetEncoders();
        updatePosition(distance, angle);
    }

    //moving backward distance (inch) with power [0, 1]
    public void backward(double power, double distance) {
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double angle = getHeading() + 180;
        int counts = -(int) ((distance / (wheelDiameter * Math.PI)) * (encoderCounts / gearRatio));

        while (counts <= robot.backLeftMotor.getCurrentPosition()) {
            setAllMotorsTo(-power);
        }
        //setting all motor powers to 0 (stopping)
        setAllMotorsTo(0);
        stopAndResetEncoders();
        updatePosition(-distance, angle);
    }

    //strafing left distance (inch) with power [0, 1]
    public void strafeLeft(double power, double distance) {
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double angle = getHeading() - 90;
        int counts = (int) ((distance / (wheelDiameter * Math.PI)) * (encoderCounts / gearRatio));
        //setting all motors to go forward (positive)
        while (counts >= robot.backLeftMotor.getCurrentPosition()) {
            setPowerOfMotorsTo(power, -power , -power , power );
        }
        //setting all motor powers to 0 (stopping)
        setAllMotorsTo(0);
        stopAndResetEncoders();
        //sleep(500);
        updatePosition(distance, angle);
    }

    //strafing right distance (inch) with power [0, 1]
    public void strafeRight(double power, double distance) {
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double angle = getHeading() + 90;
        int counts = -(int) ((distance / (wheelDiameter * Math.PI)) * (encoderCounts / gearRatio));
        while (counts <= robot.backLeftMotor.getCurrentPosition()) {
            setPowerOfMotorsTo(-power, power, power, -power);
        }
        //setting all motor powers to 0 (stopping)
        setAllMotorsTo(0);
        stopAndResetEncoders();
    }

    public void right(double power, double degrees) {

        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double initAngle = getHeading(); //angle that the robot is at when it starts

        //wait until angle turned is >= angle inputted
        while (initAngle - getHeading() <= degrees) {
            telemetry.addData("right", initAngle - getHeading());
            telemetry.update();


            //setting left motors to go forward (positive power)
            robot.backLeftMotor.setPower(power);
            robot.frontLeftMotor.setPower(power);

            //setting right motors to go backward (negative power)
            robot.backRightMotor.setPower(-power);
            robot.frontRightMotor.setPower(-power);
        }

        //setting motor value to 0 (stop)
        stopAndResetEncoders();

    }

    public void left(double power, double degrees) {

        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double initAngle = getHeading(); //angle that the robot is at when it starts

        //wait until angle turned is >= angle inputted
        while (Math.abs(getHeading() - initAngle) <= degrees) {
            telemetry.addData("left", getHeading() - initAngle);
            telemetry.update();


            //setting left motors to go backward (negitive power)
            robot.backLeftMotor.setPower(-power);
            robot.frontLeftMotor.setPower(-power);

            //setting right motors to go forward (positive power)
            robot.backRightMotor.setPower(power);
            robot.frontRightMotor.setPower(power);
        }

        //setting motor value to 0 (stop)
        stopAndResetEncoders();

    }


    public void turnRightWithPID(double targetAngle, double kp, double ki, double kd, double threshold) {
        double error = targetAngle - getHeading();
        double previousError = error;
        double correction;
        double slope;
        runtime.reset();

        while (Math.abs(error) > threshold && opModeIsActive()) {

            error = targetAngle - getHeading(); //updating error
            slope = (error - previousError) / (double) runtime.time();
            telemetry.addData("slope", slope);
            telemetry.addData("error", error);
            telemetry.update();

            runtime.reset();

            correction = (kp * error)+(kd * slope);
            setPowerOfMotorsTo(correction, correction, -correction, -correction);

        }
        setAllMotorsTo(0);
        stopAndResetEncoders();

    }

    public void turnLeftWithPID(double targetAngle, double kp, double ki, double kd, double threshold) {
        double error = targetAngle - getHeading();
        double previousError = error;
        double correction;
        double slope = 0;
        runtime.reset();

        while (Math.abs(error) > threshold && opModeIsActive()) {

            error = targetAngle - getHeading();
            slope = (error - previousError) / (double) runtime.time();
            previousError = error;
            telemetry.addData("slope", slope);
            telemetry.addData("error", error);
            telemetry.update();

            runtime.reset();

            correction = (kp * error) + (kd * slope);
            setPowerOfMotorsTo(-correction, -correction, correction, correction);

        }

    }

    //sets the power of motors to seperate inputted powers
    public void setPowerOfMotorsTo(double bl, double fl, double br, double fr) {
        robot.backLeftMotor.setPower(bl);
        robot.backRightMotor.setPower(br);
        robot.frontRightMotor.setPower(fr);
        robot.frontLeftMotor.setPower(fl);
    }

    //goes to a position and angle on the field
    public void goToPosition(double power, double x, double y, double angle) {
        double deltaX = currentxPosition - x; //required change in x position from current to desired
        double deltaY = currentyPosition - y; //required change in y position from current to desired
        double theta = Math.atan(deltaY / deltaX) + getHeading() + 45;
        double distance = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
        double xDist = Math.cos(theta) * distance;
        double yDist = Math.sin(theta) * distance;

        // setting scale factor to the biggest value
        // dividing by this when setting powers will make sure that speeds are in the format 0-1
        // will also insure speeds are proportional to distance needed to travel in each direction so robot can move at angle theta
        double scaleFactor = xDist;
        if (yDist > xDist) {
            scaleFactor = yDist;
        }
        int counts = (int) ((xDist / (wheelDiameter * Math.PI)) * (encoderCounts / gearRatio));

        setPowerOfMotorsTo(yDist / scaleFactor, xDist / scaleFactor, xDist / scaleFactor, yDist / scaleFactor);
        while(robot.backLeftMotor.getCurrentPosition()<counts){
            idle();
        }
        //setting all motor powers to 0 (stopping)
        setAllMotorsTo(0);
        stopAndResetEncoders();
        currentyPosition=y;
        currentxPosition=x;
    }

    //sets power of intake
    public void setIntakePower(double power){
        robot.intake.setPower(power);
    }

    //set servo position
    public void controlLaunchServo(double position){
        //robot.barrierServo.setPosition(position);
    }

    //set claw positiom
    public void controlClawServo(double position){
        robot.clawServo.setPosition(position);
    }

    //arm position
    public void controlArmServo(double position){
        robot.armServo.setPosition(position);
    }

    //flywheel
    public void setShooterPower(double power){
        robot.shooter.setPower(power);
    }



    //sets all motors to the same power
    public void setAllMotorsTo(double power) {
        robot.backLeftMotor.setPower(power);
        robot.backRightMotor.setPower(power);
        robot.frontRightMotor.setPower(power);
        robot.frontLeftMotor.setPower(power);
    }

    //resets all wheel encoders to 0
    public void stopAndResetEncoders() {
        robot.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    //set mode to run withouth encodrs
    public void runWithouthEncoders() {
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //updates position on the field
    public void updatePosition(double distance, double angle) {
        currentxPosition += Math.sin(angle) * distance;
        currentyPosition += Math.cos(angle) * distance;
    }

    //gets the angle in degrees
    public double getHeading() {
        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle; // left [0,-180] right[0,180]
    }














    //----------------------------------------------------------------------------------------------
    // Camera operations
    //----------------------------------------------------------------------------------------------

    private void initializeFrameQueue(int capacity) {
        /** The frame queue will automatically throw away bitmap frames if they are not processed
         * quickly by the OpMode. This avoids a buildup of frames in memory */
        frameQueue = new EvictingBlockingQueue<Bitmap>(new ArrayBlockingQueue<Bitmap>(capacity));
        frameQueue.setEvictAction(new Consumer<Bitmap>() {
            @Override public void accept(Bitmap frame) {
                // RobotLog.ii(TAG, "frame recycled w/o processing");
                frame.recycle(); // not strictly necessary, but helpful
            }
        });
    }

    private void openCamera() {
        if (camera != null) return; // be idempotent

        Deadline deadline = new Deadline(secondsPermissionTimeout, TimeUnit.SECONDS);
        camera = cameraManager.requestPermissionAndOpenCamera(deadline, cameraName, null);
        if (camera == null) {
            //error("camera not found or permission to use not granted: %s", cameraName);
        }
    }

    private void startCamera() {
        if (cameraCaptureSession != null) return; // be idempotent

        /** YUY2 is supported by all Webcams, per the USB Webcam standard: See "USB Device Class Definition
         * for Video Devices: Uncompressed Payload, Table 2-1". Further, often this is the *only*
         * image format supported by a camera */
        final int imageFormat = ImageFormat.YUY2;

        /** Verify that the image is supported, and fetch size and desired frame rate if so */
        CameraCharacteristics cameraCharacteristics = cameraName.getCameraCharacteristics();
        if (!contains(cameraCharacteristics.getAndroidFormats(), imageFormat)) {
            //error("image format not supported");
            return;
        }
        final Size size = cameraCharacteristics.getDefaultSize(imageFormat);
        final int fps = cameraCharacteristics.getMaxFramesPerSecond(imageFormat, size);

        /** Some of the logic below runs asynchronously on other threads. Use of the synchronizer
         * here allows us to wait in this method until all that asynchrony completes before returning. */
        final ContinuationSynchronizer<CameraCaptureSession> synchronizer = new ContinuationSynchronizer<>();
        try {
            /** Create a session in which requests to capture frames can be made */
            camera.createCaptureSession(Continuation.create(callbackHandler, new CameraCaptureSession.StateCallbackDefault() {
                @Override public void onConfigured(CameraCaptureSession session) {
                    try {
                        /** The session is ready to go. Start requesting frames */
                        final CameraCaptureRequest captureRequest = camera.createCaptureRequest(imageFormat, size, fps);
                        session.startCapture(captureRequest,
                                new CameraCaptureSession.CaptureCallback() {
                                    @Override public void onNewFrame( CameraCaptureSession session, CameraCaptureRequest request, CameraFrame cameraFrame) {
                                        /** A new frame is available. The frame data has <em>not</em> been copied for us, and we can only access it
                                         * for the duration of the callback. So we copy here manually. */
                                        Bitmap bmp = captureRequest.createEmptyBitmap();
                                        cameraFrame.copyToBitmap(bmp);
                                        frameQueue.offer(bmp);
                                    }
                                },
                                Continuation.create(callbackHandler, new CameraCaptureSession.StatusCallback() {
                                    @Override public void onCaptureSequenceCompleted(CameraCaptureSession session, CameraCaptureSequenceId cameraCaptureSequenceId, long lastFrameNumber) {
                                        RobotLog.ii(TAG, "capture sequence %s reports completed: lastFrame=%d", cameraCaptureSequenceId, lastFrameNumber);
                                    }
                                })
                        );
                        synchronizer.finish(session);
                    } catch (CameraException |RuntimeException e) {
                        RobotLog.ee(TAG, e, "exception starting capture");
                        //error("exception starting capture");
                        session.close();
                        synchronizer.finish(null);
                    }
                }
            }));
        } catch (CameraException|RuntimeException e) {
            RobotLog.ee(TAG, e, "exception starting camera");
            //error("exception starting camera");
            synchronizer.finish(null);
        }

        /** Wait for all the asynchrony to complete */
        try {
            synchronizer.await();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        /** Retrieve the created session. This will be null on error. */
        cameraCaptureSession = synchronizer.getValue();
    }

    private void stopCamera() {
        if (cameraCaptureSession != null) {
            cameraCaptureSession.stopCapture();
            cameraCaptureSession.close();
            cameraCaptureSession = null;
        }
    }

    private void closeCamera() {
        stopCamera();
        if (camera != null) {
            camera.close();
            camera = null;
        }
    }

    //----------------------------------------------------------------------------------------------
    // Utilities
    //----------------------------------------------------------------------------------------------

    private void error(String msg) {
        telemetry.log().add(msg);
        telemetry.update();
    }
    private void error(String format, Object...args) {
        telemetry.log().add(format, args);
        telemetry.update();
    }

    private boolean contains(int[] array, int value) {
        for (int i : array) {
            if (i == value) return true;
        }
        return false;
    }

    public int findNumRings(Bitmap bitmap) {

        //Mat region1_Cb;
        //Mat HSV = new Mat();
        //Mat Cb = new Mat();

        int rings = 0;
        Mat input = new Mat();
        Utils.bitmapToMat(bitmap, input);
        Imgproc.cvtColor(input, input, Imgproc.COLOR_BGR2HSV);
        //org.opencv.core.Size size = new org.opencv.core.Size(400,400);
        //Imgproc.resize( input, input, size);
        //Utils.matToBitmap(input, bitmap);
        File file = new File(captureDirectory, String.format(Locale.getDefault(), "webcam-frame-%d.jpg", captureCounter++));
        try {
            try (FileOutputStream outputStream = new FileOutputStream(file)) {
                bitmap.compress(Bitmap.CompressFormat.JPEG, 100, outputStream);
                //telemetry.log().add("captured %s", file.getName());
            }
        } catch (IOException e) {
            RobotLog.ee(TAG, e, "exception in saveBitmap()");
            //error("exception saving %s", file.getName());
        }
        //region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));

        //avg1 = (int) Core.mean(region1_Cb).val[0];
        //Imgproc.rectangle(input, // Buffer to draw on
        //        region1_pointA, // First point which defines the rectangle
        //        region1_pointB, // Second point which defines the rectangle
        //        BLUE, // The color the rectangle is drawn in
        //        2); // Thickness of the rectangle lines
        //position = EasyOpenCVExample.SkystoneDeterminationPipeline.RingPosition.FOUR; // Record our analysis

        Imgproc.rectangle(input, new Point(0,0), new Point(640,320), new Scalar(0,0,255), -1);
        Imgproc.rectangle(input, new Point(0,430), new Point(640,480), new Scalar(0,0,255), -1);
        Core.inRange(input, new Scalar(0, 75, 200),new Scalar(35, 230, 255),input);
        int pixels = Core.countNonZero(input);
        if(pixels > 5000){
            rings = 4;
            //position = EasyOpenCVExample.SkystoneDeterminationPipeline.RingPosition.FOUR;

        }else if (pixels > 1000){
            rings = 1;
            //position = EasyOpenCVExample.SkystoneDeterminationPipeline.RingPosition.ONE;
        }else{
            rings = 0;
            //position = EasyOpenCVExample.SkystoneDeterminationPipeline.RingPosition.NONE;
        }
//        Imgproc.rectangle(
//                input, // Buffer to draw on
//                region1_pointA, // First point which defines the rectangle
//                region1_pointB, // Second point which defines the rectangle
//                GREEN, // The color the rectangle is drawn in
//                -1); // Negative thickness means solid fill*/
        return rings;
    }
    void inputToCb(Mat input)
    {
        //Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        //Core.extractChannel(YCrCb, Cb, 1);
    }
    //@Override
    //@Override




    @Override
    public void runOpMode() throws InterruptedException {
    }
}

