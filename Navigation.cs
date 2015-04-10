using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.IO;

namespace DrRobot.JaguarControl
{
    public class Navigation
    {
        #region Navigation Variables
        public long[] LaserData = new long[DrRobot.JaguarControl.JaguarCtrl.DISDATALEN];
        public double initialX, initialY, initialT;
        public double x, y, t;
        public double x_est, y_est, t_est;
        public double desiredX, desiredY, desiredT;

        public double currentEncoderPulseL, currentEncoderPulseR;
        public double lastEncoderPulseL, lastEncoderPulseR;
        public double wheelDistanceR, wheelDistanceL;
        public double tiltAngle, zoom;
        public double currentAccel_x, currentAccel_y, currentAccel_z;
        public double lastAccel_x, lastAccel_y, lastAccel_z;
        public double currentGyro_x, currentGyro_y, currentGyro_z;
        public double last_v_x, last_v_y;
        public double filteredAcc_x, filteredAcc_y;

        public int robotType, controllerType;
        enum ROBOT_TYPE { SIMULATED, REAL };
        enum CONTROLLERTYPE { MANUALCONTROL, POINTTRACKER, EXPERIMENT };
        public bool motionPlanRequired, displayParticles, displayNodes, displaySimRobot;
        private JaguarCtrl jaguarControl;
        private AxDRROBOTSentinelCONTROLLib.AxDDrRobotSentinel realJaguar;
        private AxDDrRobotSentinel_Simulator simulatedJaguar;
        private Thread controlThread;
        private short motorSignalL, motorSignalR;
        private short desiredRotRateR, desiredRotRateL;
        public bool runThread = true;
        public bool loggingOn;
        StreamWriter logFile;
        public int deltaT = 10;

        public double deltaTReal = 10; //Lab 3 variable
        public int pointIndex = 0; //Lab 3 trajectory variable
        
        private static int encoderMax = 32767;
        public int pulsesPerRotation = 190;
        public double wheelRadius = 0.089;
        public double robotRadius = 0.242;//0.232
        private double angleTravelled, distanceTravelled;
        private double diffEncoderPulseL, diffEncoderPulseR;
        private double maxVelocity = 0.25;
        private double Kpho = 1; // originally 1 but 0 < Kpho < 0.25 to drive at correct v
        private double Kalpha = 2;//2//8
        private double Kbeta = -0.5;//-0.5//-1.0;
        const double alphaTrackingAccuracy = 0.10;
        const double betaTrackingAccuracy = 0.1;
        const double phoTrackingAccuracy = 0.10;
        double time = 0;
        DateTime startTime;
        // Lab 3 PID Time Variables
        DateTime PIDtime = DateTime.Now;

        public short K_P = 15;//15;
        public short K_I = 0;//0;
        public short K_D = 3;//3;
        public short frictionComp = 8750;//8750;
        public double e_sum_R, e_sum_L;
        public double u_R = 0;
        public double u_L = 0;
        public double e_R = 0;
        public double e_L = 0;
        public double e_L_last = 0; // Lab 3
        public double e_R_last = 0; // Lab 3

        public double accCalib_x = 18;
        public double accCalib_y = 4;

        #endregion


        #region Navigation Setup
        
        // Constructor for the Navigation class
        public Navigation(JaguarCtrl jc)
        {
            // Initialize vars
            jaguarControl = jc;
            realJaguar = jc.realJaguar;
            simulatedJaguar = jc.simulatedJaguar;
            this.Initialize();


            // Start Control Thread
            controlThread = new Thread(new ThreadStart(runControlLoop));
            controlThread.Start();
        }

        // All class variables are initialized here
        // This is called every time the reset button is pressed
        public void Initialize()
        {
            // Initialize state estimates
            x = 0;//initialX;
            y = 0;//initialY;
            t = 0;//initialT;

            // Initialize state estimates
            x_est = 0;//initialX;
            y_est = 0;//initialY;
            t_est = 0;//initialT;

            // Set desired state
            desiredX = 0;// initialX;
            desiredY = 0;// initialY;
            desiredT = 0;// initialT;

            // Reset Lab 3 Trajectory Variable
            pointIndex = 0;

            // Reset Localization Variables
            wheelDistanceR = 0;
            wheelDistanceL = 0;

            // Zero actuator signals
            motorSignalL = 0;
            motorSignalR = 0;
            loggingOn = false;

            // Set random start for particles
            //InitializeParticles();

            // Set default to no motionPlanRequired
            motionPlanRequired = false;

            // Set visual display
            tiltAngle = 25.0;
            displayParticles = true;
            displayNodes = true;
            displaySimRobot = true;

            // Lab 3 PID Time Variables
           DateTime PIDtime = DateTime.Now;
        }

        // This function is called from the dialogue window "Reset Button"
        // click function. It resets all variables.
        public void Reset()
        {
            simulatedJaguar.Reset();
            GetFirstEncoderMeasurements();
            CalibrateIMU();
            Initialize();
        }
        #endregion


        #region Main Loop

        /************************ MAIN CONTROL LOOP ***********************/
        // This is the main control function called from the control loop
        // in the RoboticsLabDlg application. This is called at every time
        // step.
        // Students should choose what type of localization and control 
        // method to use. 
        public void runControlLoop()
        {
            // Wait
            Thread.Sleep(500);

            // Don't run until we have gotten our first encoder measurements to difference with
            GetFirstEncoderMeasurements();

            // Run infinite Control Loop
            while (runThread)
            {
                // ****************** Additional Student Code: Start ************

                // Students can select what type of localization and control
                // functions to call here. For lab 1, we just call the function
                // WallPositioning to have the robot maintain a constant distance
                // to the wall (see lab manual).
                
                // Update Sensor Readings
                UpdateSensorMeasurements();

                // Determine the change of robot position, orientation (lab 2)	
                MotionPrediction();

                // Update the global state of the robot - x,y,t (lab 2)
                LocalizeRealWithOdometry();

                // Update the global state of the robot - x,y,t (lab 2)
                //LocalizeRealWithIMU();
                

                // Estimate the global state of the robot -x_est, y_est, t_est (lab 4)
                LocalizeEstWithParticleFilter();


                // If using the point tracker, call the function
                if (jaguarControl.controlMode == jaguarControl.AUTONOMOUS)
                {

                    // Check if we need to create a new trajectory
                    if (motionPlanRequired)
                    {
                        // Construct a new trajectory (lab 5)
                        PRMMotionPlanner();
                        motionPlanRequired = false;
                    }

                    // Drive the robot to 1meter from the wall. Otherwise, comment it out after lab 1. 
                    //WallPositioning();

                    // Drive the robot to a desired Point (lab 3)
                    FlyToSetPoint();

                    // Follow the trajectory instead of a desired point (lab 3)
                    //TrackTrajectory();

                    // Actuate motors based actuateMotorL and actuateMotorR
                    if (jaguarControl.Simulating())
                    {
                        CalcSimulatedMotorSignals();
                        ActuateMotorsWithVelControl();
                    }
                    else 
                    {
                        // Determine the desired PWM signals for desired wheel speeds
                        CalcMotorSignals();
                        ActuateMotorsWithPWMControl();
                    }

                }
                else
                {
                    e_sum_L = 0;
                    e_sum_R = 0;
                }
                
                // ****************** Additional Student Code: End   ************

                // Log data
                LogData();

                // Sleep to approximate 20 Hz update rate
                Thread.Sleep(deltaT); //not sure if this works anymore..... -wf
            }
        }


        public void CalibrateIMU()
        {

            accCalib_x = 0;
            accCalib_y = 0;
            int numMeasurements = 100;
            for (int i = 0; i < numMeasurements; i++)
            {
                accCalib_x += currentAccel_x;
                accCalib_y += currentAccel_y;

                Thread.Sleep(deltaT);
            }
            accCalib_x = accCalib_x / numMeasurements;
            accCalib_y = accCalib_y /numMeasurements;


        }


        // Before starting the control loop, the code checks to see if 
        // the robot needs to get the first encoder measurements
        public void GetFirstEncoderMeasurements()
        {
            if (!jaguarControl.Simulating())
            {
                // Get last encoder measurements
                bool gotFirstEncoder = false;
                int counter = 0;
                while (!gotFirstEncoder && counter < 10)
                {
                    try
                    {
                        currentEncoderPulseL = jaguarControl.realJaguar.GetEncoderPulse4();
                        currentEncoderPulseR = jaguarControl.realJaguar.GetEncoderPulse5();
                        lastEncoderPulseL = currentEncoderPulseL;
                        lastEncoderPulseR = currentEncoderPulseR;
                        gotFirstEncoder = true;

                        currentAccel_x = jaguarControl.getAccel_x();
                        currentAccel_y = jaguarControl.getAccel_y();
                        currentAccel_z = jaguarControl.getAccel_z();
                        lastAccel_x = currentAccel_x;
                        lastAccel_y = currentAccel_y;
                        lastAccel_z = currentAccel_z;
                        last_v_x = 0;
                        last_v_y = 0;

                    }
                    catch (Exception e) { }
                    counter++;
                    Thread.Sleep(100);
                }
            }
            else
            {
                currentEncoderPulseL = 0;
                currentEncoderPulseR = 0;
                lastEncoderPulseL = 0;
                lastEncoderPulseR = 0;
                lastAccel_x = 0;
                lastAccel_y = 0;
                lastAccel_z = 0;
                last_v_x = 0;
                last_v_y = 0;

            }
        }

        // At every iteration of the control loop, this function will make 
        // sure all the sensor measurements are up to date before
        // makeing control decisions.
        public void UpdateSensorMeasurements()
        {
            // For simulations, update the simulated measurements
            if (jaguarControl.Simulating())
            {
                jaguarControl.simulatedJaguar.UpdateSensors(deltaT);

                // Get most recenct encoder measurements
                currentEncoderPulseL = simulatedJaguar.GetEncoderPulse4();
                currentEncoderPulseR = simulatedJaguar.GetEncoderPulse5();
            }
            else
            {
                // Get most recenct encoder measurements
                try
                {

                    // Update IMU Measurements
                    currentAccel_x = jaguarControl.getAccel_x();
                    currentAccel_y = jaguarControl.getAccel_y();
                    currentAccel_z = jaguarControl.getAccel_z();
                   
                    // Update Encoder Measurements
                    currentEncoderPulseL = jaguarControl.realJaguar.GetEncoderPulse4();
                    currentEncoderPulseR = jaguarControl.realJaguar.GetEncoderPulse5();

                }
                catch (Exception e)
                {
                }
            }
        }

        // At every iteration of the control loop, this function calculates
        // the PWM signal for corresponding desired wheel speeds
        public void CalcSimulatedMotorSignals()
        {

            motorSignalL = (short)(desiredRotRateL);
            motorSignalR = (short)(desiredRotRateR);

        }

        public void CalcMotorSignals()
        {
            TimeSpan PIDts = DateTime.Now - PIDtime; // time span between each PID function call
            deltaTReal = PIDts.Milliseconds; // / 1000.0; // update deltaTReal with only the integer milliseconds, replaces deltaT
            Console.WriteLine("Time Difference: " + deltaTReal);
            PIDtime = DateTime.Now; // update PID time

            short zeroOutput = 16383;
            short maxPosOutput = 32767;

            double K_p = 40; //originally 25
            double K_i = 0.001; //originally 0.1
            double K_d = 0.5; //originally 1
            
            double maxErr = 8000 / deltaTReal;

            // desiredRotRate is desired speed in encoder pulses per second
            // diffEncoderPulse / deltaTReal is the actual speed in pulses/s
            e_L = desiredRotRateL - diffEncoderPulseL / deltaTReal;
            e_R = desiredRotRateR - diffEncoderPulseR / deltaTReal;

            // why is it 0.9?
            e_sum_L = .9 * e_sum_L + e_L * deltaTReal;
            e_sum_R = .9 * e_sum_R + e_R * deltaTReal;

            e_sum_L = Math.Max(-maxErr, Math.Min(e_sum_L, maxErr));
            e_sum_R = Math.Max(-maxErr, Math.Min(e_sum_R, maxErr));

            u_L = ((K_p * e_L) + (K_i * e_sum_L) + (K_d * (e_L - e_L_last) / deltaTReal));
            e_L_last = e_L;

            u_R = ((K_p * e_R) + (K_i * e_sum_R) + (K_d * (e_R - e_R_last) / deltaTReal));
            e_R_last = e_R;
            
            // The following settings are used to help develop the controller in simulation.
            // They will be replaced when the actual jaguar is used.
            //motorSignalL = (short)(zeroOutput + desiredRotRateL * 100);// (zeroOutput + u_L);
            //motorSignalR = (short)(zeroOutput - desiredRotRateR * 100);//(zeroOutput - u_R);

            motorSignalL = (short)((zeroOutput + u_L)); // I added the 3000 here
            motorSignalR = (short)((zeroOutput - u_R)); // I added the 3000 here

            motorSignalL = (short)Math.Min(maxPosOutput, Math.Max(0, (int)motorSignalL));
            motorSignalR = (short)Math.Min(maxPosOutput, Math.Max(0, (int)motorSignalR));

            //ZERO OUTPUT VELOCITIES (robot won't move at this speed)
            //motorSignalL = zeroOutput;
            //motorSignalR = zeroOutput;

            //Console.WriteLine("Desired Rot Rate: " + desiredRotRateL + "Diff Enc Pulse: " + diffEncoderPulseL + ", " + diffEncoderPulseR + " Error: " + e_L + " Motor Signal: " + motorSignalL);
            //Console.WriteLine("Speed: " + 100.0 * ((double) (motorSignalL - zeroOutput) / (double) zeroOutput) + " %");

        }

        // At every iteration of the control loop, this function sends
        // the width of a pulse for PWM control to the robot motors
        public void ActuateMotorsWithPWMControl()
        { 
            if (jaguarControl.Simulating())
                simulatedJaguar.DcMotorPwmNonTimeCtrAll(0, 0, 0, motorSignalL, motorSignalR, 0);
            else
            {
                jaguarControl.realJaguar.DcMotorPwmNonTimeCtrAll(0, 0, 0, motorSignalL, motorSignalR, 0);
                //BIG NOTE: motorSignal is a duty cycle, so 16383 would be 50% duty cycle;
                //0 & 32000 are fastest, close to 16383 is slow; 20,000 is a good slow value
            }
        }

        // At every iteration of the control loop, this function sends
        // desired wheel velocities (in pulses / second) to the robot motors
        public void ActuateMotorsWithVelControl()
        {
            if (jaguarControl.Simulating())
                simulatedJaguar.DcMotorVelocityNonTimeCtrAll(0, 0, 0, motorSignalL, (short)(-motorSignalR), 0);
            else
            {
                // Setup Control
                jaguarControl.realJaguar.SetDcMotorVelocityControlPID(3, K_P, K_D, K_I);
                jaguarControl.realJaguar.SetDcMotorVelocityControlPID(4, K_P, K_D, K_I);

                jaguarControl.realJaguar.DcMotorVelocityNonTimeCtrAll(0, 0, 0, motorSignalL, (short)(-motorSignalR), 0);
            }
        }
        #endregion


        #region Logging Functions

        // This function is called from a dialogue window "Record" button
        // It creates a new file and sets the logging On flag to true
        public void TurnLoggingOn()
        {
            //int fileCnt= 0;
            String date = DateTime.Now.Year.ToString() + "-" + DateTime.Now.Month.ToString() + "-" + DateTime.Now.Day.ToString() + "-" + DateTime.Now.Minute.ToString();
            ToString();
            logFile = File.CreateText("JaguarData_" + date + ".csv");
            startTime = DateTime.Now;
            loggingOn = true;
        }

        // This function is called from a dialogue window "Record" button
        // It closes the log file and sets the logging On flag to false
        public void TurnLoggingOff()
        {
            if (logFile != null)
                logFile.Close();
            loggingOn = false;
        }

        // This function is called at every iteration of the control loop
        // IF the loggingOn flag is set to true, the function checks how long the 
        // logging has been running and records this time
        private void LogData()
        {
            if (loggingOn)
            {
                TimeSpan ts = DateTime.Now - startTime;
                time = ts.TotalSeconds;
                 // String newData = time.ToString() + "," + x.ToString() + "," + y.ToString() + "," + t.ToString() + "," + u_L.ToString() + "," + u_R.ToString();
                double u_avg = ((double)(u_L + u_R)) / 2.0;
                //double speed_prcnt = (double)((((double)(motorSignalL + motorSignalR) / 2.0) - 16383.0) / 16383.0);
                String newData = time.ToString() + "," + x.ToString(); // u_L.ToString() + "," + u_R.ToString();// +"," + desiredRotRateL + "," + desiredRotRateR + "," + e_L + "," + e_R + "," + diffEncoderPulseL + "," + diffEncoderPulseR + "," + motorSignalL + "," + motorSignalR;
                logFile.WriteLine(newData);
            }
        }
        #endregion


        # region Control Functions

        // This function is called at every iteration of the control loop
        // It will drive the robot forward or backward to position the robot 
        // 1 meter from the wall.
        private void WallPositioning()
        {

            // Here is the distance measurement for the central laser beam 
            double centralLaserRange = LaserData[113];

            // ****************** Additional Student Code: Start ************

            // Put code here to calculated motorSignalR and 
            // motorSignalL. Make sure the robot does not exceed 
            // maxVelocity!!!!!!!!!!!!

            // Send Control signals, put negative on left wheel control

 

            // ****************** Additional Student Code: End   ************                
        }


        // This function is called at every iteration of the control loop
        // if used, this function can drive the robot to any desired
        // robot state. It does not check for collisions
        private void FlyToSetPoint()
        {

            // ****************** Additional Student Code: Start ************

            // Put code here to calculate motorSignalR and 
            // motorSignalL. Make sure the robot does not exceed 
            // maxVelocity!!!!!!!!!!!!

            // Part 1: Transform to new coordinate system

            double deltaX = desiredX - x_est;
            double deltaY = desiredY - y_est;
            double theta = t_est - 0; // theta is the current heading relative to 0

            // clean deltaX and deltaY to remove oscillations
            // yhis will ignore the delta overshoot between two function call
            deltaX = (Math.Abs(deltaX) < 0.001) ? 0 : deltaX;
            deltaY = (Math.Abs(deltaY) < 0.001) ? 0 : deltaY;

            double pho = Math.Sqrt(Math.Pow(deltaX, 2) + Math.Pow(deltaY, 2));
            double alpha = -theta + Math.Atan2(deltaY, deltaX);
            //double alpha = angleDifference(-theta, -Math.Atan2(deltaX, deltaY));
            double direction = 1; // default is forward motion

            // if alpha in [-pi/2, pi/2] move forward with default alpha calculation
            if ((-Math.PI / 2) <= alpha && alpha <= (Math.PI / 2))
            {
                alpha = -angleDifference(theta, Math.Atan2(deltaY, deltaX));
                direction = 1;
            }
            else // else, alpha in ]-pi, pi/2[ U ]pi/2, pi[
            {
                alpha = -angleDifference(theta, Math.Atan2(-deltaY, -deltaX));
                direction = -1;
            }


            // angle to move at goal
            double beta = angleDifference(-theta, alpha);
            beta = angleDifference(beta, -desiredT);

            // Part 2: Calculate Desired Wheel Velocities

            // will experiment with Kphi, Kalpha, and Kbeta
            double desiredV = direction * Kpho * pho; // correct for forward vs backward motion
            double desiredW = Kalpha * alpha + Kbeta * beta;

            // fix for in-place rotation
            //if (Math.Abs(Math.Abs(theta) - Math.Abs(alpha)) < 0.1) desiredW = -Kbeta/2.5 * desiredT; // Kbeta < 0
            if (pho < 0.1) desiredW = -2 * angleDifference(theta, desiredT); // experiental coef

            desiredRotRateL = (short)(pulsesPerRotation / (2 * Math.PI * wheelRadius) * (desiredV + desiredW * robotRadius)); // enc. pulses / sec
            desiredRotRateR = (short)(pulsesPerRotation / (2 * Math.PI * wheelRadius) * (desiredV - desiredW * robotRadius)); // enc. pulses / sec

            // Limit wheel velocities to maxVelocity (0.25 m/s)
            short maxRotRate = (short)(3 * maxVelocity * pulsesPerRotation / (2 * Math.PI * wheelRadius)); // 0.25 * 190/(2pi * 0.089) = 84 enc. pulses / sec
            int rotDirL = (desiredRotRateL >= 0) ? 1 : -1;
            int rotDirR = (desiredRotRateR >= 0) ? 1 : -1;
            double rotRatio = Math.Abs(desiredRotRateL / (double)desiredRotRateR);
            if (Math.Abs(desiredRotRateR) > maxRotRate || Math.Abs(desiredRotRateL) > maxRotRate)
            {
                if (rotRatio > 1)
                {
                    desiredRotRateL = (short)(rotDirL * maxRotRate);
                    desiredRotRateR = (short)(rotDirR * maxRotRate / rotRatio);
                }
                else
                {
                    desiredRotRateR = (short)(rotDirR * maxRotRate);
                    desiredRotRateL = (short)(rotDirL * maxRotRate * rotRatio);
                }
            }

            //fix for oscillations when arrived at destination
            if (pho < 0.1 && Math.Abs(angleDifference(theta, desiredT)) < 0.02)
            {
                //desiredRotRateL = (desiredRotRateL < 1) ? (short) 0 : desiredRotRateL;
                //desiredRotRateR = (desiredRotRateR < 1) ? (short) 0 : desiredRotRateR;
                desiredRotRateL = (short)0;
                desiredRotRateR = (short)0;
            }

            // ****************** Additional Student Code: End   ************
        }


        // get the shortest angle difference in radians
        private double angleDifference(double source, double target)
        {
            double diff = (source - target) % (2*Math.PI);
            if (diff > Math.PI)
                diff = diff - 2*Math.PI;
            else if (diff < -Math.PI)
                diff = diff + 2*Math.PI;
            return diff;
        }

        // THis function is called to follow a trajectory constructed by PRMMotionPlanner()
        private void TrackTrajectory()
        {

            // Follow the x-axis forever
            //desiredX = desiredX + 0.2;
            //desiredY = 0;
            //desiredT = 0;

            // Follow the x-axis up to (1,0) the go up forever
            //if (x_est < 1)
            //{
            //    desiredX = desiredX + 0.01;
            //} else if ( t_est < 1.57 && y_est < 0.5) // turn +pi/2
            //{
            //    desiredX = 1;
            //    desiredT = desiredT + 0.01;
            //}
            //else
            //{
            //    desiredX = 1;
            //    desiredY = desiredY + 0.01;
            //    desiredT = 1.57;
            //}

            // follow a curve NOT WORKING
            //desiredX = trajectoryTime/600;
            //desiredY = trajectoryTime/400;
            //desiredT = t_est;

            // circle NOT WORKING
            //desiredX = Math.Cos(x_est*0.5+0.1);
            //desiredY = Math.Sin(0.5*y_est+0.1);

            // Lab 3 Code

            // Follow a set of points
            double[,] points = {{0.5, 0}, {1, 0}, {1.5, 0}, {2,0}, {2, 0.5}, {1.5, 1}, {1, 1.5}, {0,2}, {-0.5, 1.5}, {-1, 1}, {-1.5, 0.5}, {-2, 0}, {-1.5, -0.5}, {-1, -1}, {-0.5, -1.5}, {0, -2}, {0.5, -1.5}, {1, -1}, {1.5, -0.5}, {2, 0}};

            double deltaX = desiredX - x_est;
            double deltaY = desiredY - y_est;
            double deltaXtoNext = 0;
            double deltaYtoNext = 0;

            if (pointIndex < points.Length / 2 - 1)
            {
                deltaXtoNext = points[pointIndex + 1, 0] - points[pointIndex, 0];
                deltaYtoNext = points[pointIndex + 1, 1] - points[pointIndex, 1];
              //  desiredT = Math.Atan2(deltaYtoNext, deltaXtoNext);
            }
            //else { desiredT = 0; }
        

            desiredX = points[pointIndex, 0];
            desiredY = points[pointIndex, 1];
            

            double distanceToPoint = Math.Sqrt(Math.Pow(deltaX, 2) + Math.Pow(deltaY, 2));
            Console.WriteLine(distanceToPoint);
            if (distanceToPoint < 0.1 && pointIndex < points.Length/2 - 1 )  pointIndex++;

            FlyToSetPoint();
        }

        // THis function is called to construct a collision-free trajectory for the robot to follow
        private void PRMMotionPlanner()
        {

        }


        #endregion


        #region Localization Functions
        /************************ LOCALIZATION ***********************/

        // This function will grab the most recent encoder measurements
        // from either the simulator or the robot (whichever is activated)
        // and use those measurements to predict the RELATIVE forward 
        // motion and rotation of the robot. These are referred to as
        // distanceTravelled and angleTravelled respectively.
        public void MotionPrediction()//CWiRobotSDK* m_MOTSDK_rob)
        {

            // ****************** Additional Student Code: Start ************

            // Put code here to calculated distanceTravelled and angleTravelled.
            // You can set and use variables like diffEncoder1, currentEncoderPulse1,
            // wheelDistanceL, wheelRadius, encoderResolution etc. These are defined
            // in the Robot.h file.

            /// Code from Lab 2

            // Difference in encoder values for each wheel
            diffEncoderPulseL = currentEncoderPulseL - lastEncoderPulseL;
            diffEncoderPulseR = -currentEncoderPulseR + lastEncoderPulseR;

            // The encoder has a range of 0 to encoderMax (32763)
            double encoderDiffMax = encoderMax / 2; // Max value for difference in encoder values

            // Accounting for encoder rollover and detecting backward vs forward motion:

            if (diffEncoderPulseL > encoderDiffMax) //Difference is too positive 
            {
                diffEncoderPulseL = diffEncoderPulseL - encoderMax;
            }
            else if (diffEncoderPulseL < -encoderDiffMax) //Difference is too negative
            {
                diffEncoderPulseL = diffEncoderPulseL + encoderMax;
            }

            if (diffEncoderPulseR > encoderDiffMax) //Difference is too positive
            {
                diffEncoderPulseR = diffEncoderPulseR - encoderMax;
            }
            else if (diffEncoderPulseR < -encoderDiffMax) //Differnce is too negative
            {
                diffEncoderPulseR = diffEncoderPulseR + encoderMax;
            }

            // encoderResolution is 190 counts/rev
            double encoderResolution = pulsesPerRotation;

            // Distance traveled by each wheel (since the encoders have 190 counts/rev)
            wheelDistanceL = 2 * Math.PI * wheelRadius * (diffEncoderPulseL / encoderResolution);
            wheelDistanceR = 2 * Math.PI * wheelRadius * (diffEncoderPulseR / encoderResolution);

            // Average distance traveled by each wheel to get distance traveled by robot
            distanceTravelled = (wheelDistanceL + wheelDistanceR) / 2;

            // Angle traveled is difference in distances traveled by each wheel / 2*L
            // Angle traveled should be within -pi and pi
            angleTravelled = (wheelDistanceL - wheelDistanceR) / (2 * robotRadius);

            // Calculate estimated states x_est, y_est, t_test
            x_est = x + distanceTravelled * Math.Cos(t + angleTravelled / 2);
            y_est = y + distanceTravelled * Math.Sin(t + angleTravelled / 2);
            t_est = t + angleTravelled;

            // Keep angle of robot within -pi and pi
            if (t_est >= Math.PI) // if angle is over pi
            {
                t_est = (t_est % Math.PI) - Math.PI; //roll over to -pi to 0 range
            }
            if (t_est <= -Math.PI) // if angle is less than pi
            {
                t_est = (t_est % Math.PI) + Math.PI; //roll over to 0 to pi range
            }

            // Update last encoder count variables
            lastEncoderPulseL = currentEncoderPulseL;
            lastEncoderPulseR = currentEncoderPulseR;
    

            // ****************** Additional Student Code: End   ************
        }
 


        // This function will Localize the robot, i.e. set the robot position
        // defined by x,y,t using the last position with angleTravelled and
        // distance travelled.
        public void LocalizeRealWithOdometry()//CWiRobotSDK* m_MOTSDK_rob)
        {
            // ****************** Additional Student Code: Start ************

            // Put code here to calculate x,y,t based on odemetry 
            // (i.e. using last x, y, t as well as angleTravelled and distanceTravelled).
            // Make sure t stays between pi and -pi

            // Lab 2 Code

            x = x_est;
            y = y_est;
            t = t_est;

            // ****************** Additional Student Code: End   ************
        }
        // This function will Localize the robot, i.e. set the robot position
        // defined by x,y,t using the last position with angleTravelled and
        // distance travelled.
        public void LocalizeRealWithIMU()//CWiRobotSDK* m_MOTSDK_rob)
        {
            // ****************** Additional Student Code: Start ************

            // Put code here to calculate x,y,t based on odemetry 
            // (i.e. using last x, y, t as well as angleTravelled and distanceTravelled).
            // Make sure t stays between pi and -pi


            // ****************** Additional Student Code: End   ************
        }


        public void LocalizeEstWithParticleFilter()
        {
            // To start, just set the estimated to be the actual for simulations
            // This will not be necessary when running the PF lab
            x_est = x;
            y_est = y;
            t_est = t;

            // ****************** Additional Student Code: Start ************

            // Put code here to calculate x_est, y_est, t_est using a PF




            // ****************** Additional Student Code: End   ************

        }
        #endregion

    }
}
