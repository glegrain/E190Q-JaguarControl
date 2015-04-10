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
        public double x_des, y_des, t_des;
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
        private static int encoderMax = 32767;
        public int pulsesPerRotation = 190;
        public double wheelRadius = 0.089;
        public double robotRadius = 0.242;//0.232
        private double angleTravelled, distanceTravelled;
        private double diffEncoderPulseL, diffEncoderPulseR;
        private double maxVelocity = 0.25;
        private double Kpho = 1;
        private double Kalpha = 2;//8
        private double Kbeta = -0.5;//-0.5//-1.0;
        const double alphaTrackingAccuracy = 0.10;
        const double betaTrackingAccuracy = 0.1;
        const double phoTrackingAccuracy = 0.10;
        double time = 0;
        DateTime startTime;

        public short K_P = 15;//15;
        public short K_I = 0;//0;
        public short K_D = 3;//3;
        public short frictionComp = 8750;//8750;
        public double e_sum_R, e_sum_L;
        public double u_R = 0;
        public double u_L = 0;
        public double e_R = 0;
        public double e_L = 0;

        public double rotRateL, rotRateR;
        public double K_p, K_i, K_d, maxErr;

        public double accCalib_x = 18;
        public double accCalib_y = 4;

        // PF Variables
        public Map map;
        public Particle[] particles;
        public Particle[] propagatedParticles;
        public int numParticles = 8000;
        public double K_wheelRandomness = 0.15;//0.25
        public Random random = new Random();
        public bool newLaserData = false;
        public double laserMaxRange = 4.0;
        public double laserMinRange = 0.2;
        public double[] laserAngles;
        private int laserCounter;
        private int laserStepSize = 3;

        // Lab 4 Particle variable
        public double varxx = 0;
        public double varyy = 0;
        public double varxy = 0;

        public class Particle
        {
            public double x, y, t, w;

            public Particle()
            {
            }
            
            // Lab 4; our own copy method
            public Particle copy()
            {
                Particle copy = new Particle();
                copy.x = this.x;
                copy.y = this.y;
                copy.t = this.t;
                copy.w = this.w;
                return copy;
            }
        }

        // Motion Planner Variables
        const int numXCells = 20;
        const int numYCells = 20;
        const int maxNumNodes = 5000;
        const float minWorkspaceX = -10.0f;
        const float maxWorkspaceX = 10.0f;
        const float minWorkspaceY = -10.0f;
        const float maxWorkspaceY = 10.0f;

        // Motion Planner Variables 
        public double samplingCellSizeX, samplingCellSizeY;
        public int numOccupiedCells;
        public int[] occupiedCellsList;
        public int[] numNodesInCell;
        public Node[,] NodesInCells;
        public Node[] trajList, nodeList;
        public int trajSize, trajCurrentNode, numNodes;

        public class Node
        {
            public double x, y;
            public int lastNode;
            public int nodeIndex;

            public Node()
            {
                x = 0;
                y = 0;
                lastNode = 0;
                nodeIndex = 0;
            }

            public Node(double _x, double _y, int _nodeIndex, int _lastNode)
            {
                x = _x;
                y = _y;
                nodeIndex = _nodeIndex;
                lastNode = _lastNode;
            }
        }


        #endregion


        #region Navigation Setup
        
        // Constructor for the Navigation class
        public Navigation(JaguarCtrl jc)
        {
            // Initialize vars
            jaguarControl = jc;
            realJaguar = jc.realJaguar;
            simulatedJaguar = jc.simulatedJaguar;
            map = new Map();

            // Create particles
            particles = new Particle[numParticles];
            propagatedParticles = new Particle[numParticles];
            for (int i = 0; i < numParticles; i++)
            {
                particles[i] = new Particle();
                propagatedParticles[i] = new Particle();
            }

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

            // Reset Localization Variables
            wheelDistanceR = 0;
            wheelDistanceL = 0;

            // Zero actuator signals
            motorSignalL = 0;
            motorSignalR = 0;
            loggingOn = false;

            // Set random start for particles
            InitializeParticles();

            // Set default to no motionPlanRequired
            motionPlanRequired = false;

            // Set visual display
            tiltAngle = 25.0;
            displayParticles = true;
            displayNodes = true;
            displaySimRobot = true;

            laserAngles = new double[LaserData.Length];
            for (int i = 0; i < LaserData.Length; i++)                
                laserAngles[i] = DrRobot.JaguarControl.JaguarCtrl.startAng + DrRobot.JaguarControl.JaguarCtrl.stepAng * i;

            // MP variable setup
            occupiedCellsList = new int[numXCells * numYCells];
            numNodesInCell = new int[numXCells * numYCells];
            NodesInCells = new Node[numXCells * numYCells, 500];
            trajList = new Node[maxNumNodes];
            nodeList = new Node[maxNumNodes];
            numNodes = 0;
            trajList[0] = new Node(0, 0, 0, 0);
            trajSize = 0;


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

                    // Drive the robot to a desired Point (lab 3)
                    //FlyToSetPoint();

                    // Follow the trajectory instead of a desired point (lab 3)
                    TrackTrajectory();

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

                // Get most recent laser scanner measurements
                laserCounter = laserCounter + deltaT;
                if (laserCounter >= 2000)
                {
                    for (int i = 0; i < LaserData.Length; i=i+laserStepSize)
                    {
                        LaserData[i] = (long)(1000 * map.GetClosestWallDistance(x, y, t -1.57 + laserAngles[i]));
                    }
                    laserCounter = 0;
                    newLaserData = true;
                }
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
            // OLD CODE
            //short zeroOutput = 16383;

            //// ADD STUDENT CODE HERE //////////////////////////


            //motorSignalL = (short)(zeroOutput + 0);
            //motorSignalR = (short)(zeroOutput - 0);

            // ADDED FROM PREVIOUS LABS

            short zeroOutput = 16383;
            short maxPosOutput = 32767;

            motorSignalL = (short)Math.Min(maxPosOutput, Math.Max(0, (int)motorSignalL));
            motorSignalR = (short)Math.Min(maxPosOutput, Math.Max(0, (int)motorSignalR));
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
            }
        }

        // At every iteration of the control loop, this function sends
        // desired wheel velocities (in pulses / second) to the robot motors
        public void ActuateMotorsWithVelControl()
        {
            if (jaguarControl.Simulating())
                simulatedJaguar.DcMotorVelocityNonTimeCtrAll(0, 0, 0, motorSignalL, (short)(-motorSignalR), 0);
            else
                jaguarControl.realJaguar.DcMotorVelocityNonTimeCtrAll(0, 0, 0, motorSignalL, (short)(-motorSignalR), 0);
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
            //logFile.WriteLine("time, x, y, x_est, y_est, var_xx, var_yy, var_xy"); //LAB 4
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
                 String newData = time.ToString() + " " + x.ToString() + " " + y.ToString() + " " + t.ToString() ;
                 //String newData = time.ToString() + "," + x.ToString() + "," + y.ToString() + "," + x_est.ToString() + "," + y_est.ToString() + "," + varxx.ToString() + "," + varyy.ToString() + "," + varxy.ToString(); // lab 4
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

            // Put code here to calculate desiredRotRateR and 
            // desiredRotRateL. Make sure the robot does not exceed 
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
            double diff = (source - target) % (2 * Math.PI);
            if (diff > Math.PI)
                diff = diff - 2 * Math.PI;
            else if (diff < -Math.PI)
                diff = diff + 2 * Math.PI;
            return diff;
        }

        // THis function is called to follow a trajectory constructed by PRMMotionPlanner()
        private void TrackTrajectory()
        {
            double distToCurrentNode = Math.Sqrt(Math.Pow(x_est - trajList[trajCurrentNode].x, 2) + Math.Pow(y_est - trajList[trajCurrentNode].y, 2));
            if (distToCurrentNode < 0.1 && trajCurrentNode + 1 < trajSize)
            {
                trajCurrentNode++;
                x_des = trajList[trajCurrentNode].x;
                y_des = trajList[trajCurrentNode].y;
                t_des = 0;
            }

            FlyToSetPoint();
        }

        // This function houses the core motion planner. This function
        // will generate a new trajectory when called. Students must 
        // add their code here.

        private void PRMMotionPlanner()
        {
            // Initialize sampling grid cell variables for weighted
            // random selection of nodes to expand.
            samplingCellSizeX = (maxWorkspaceX - minWorkspaceX) / numXCells;
            samplingCellSizeY = (maxWorkspaceY - minWorkspaceY) / numYCells;
            numOccupiedCells = 0;
            for (int i = 0; i < numXCells * numYCells; i++)
                numNodesInCell[i] = 0;
            numNodes = 0;


            // ****************** Additional Student Code: Start ************

            // Put code here to expand the PRM until the goal node is reached,
            // or until a max number of iterations is reached.


            // Create and add the start Node


            // Create the goal node


            // Loop until path created
            bool pathFound = false;
            int maxIterations = maxNumNodes;
            int iterations = 0;
            Random randGenerator = new Random();


            

            while (iterations < maxIterations && !pathFound)
            {

                

                // Increment number of iterations
                iterations++;
            }


            // Create the trajectory to follow
            //BuildTraj(goalNode);

            
            // ****************** Additional Student Code: End   ************




        }




        // This function is used to implement weighted sampling in 
        // when randomly selecting nodes to expand from in the PRM.
        // The work environment is divided into a grid of cells.
        // This function returns the cell number.
        int GetCellNumber(double x, double y)
        {
            int cell = (int)Math.Floor((x - minWorkspaceX) / samplingCellSizeX) + (int)(Math.Floor((y - minWorkspaceY) / samplingCellSizeY) * numXCells);
            return cell;
        }

        // This function is also used to implement weighted sampling in 
        // when randomly selecting nodes to expand from in the PRM.
        // When new nodes for the PRM are generated, they must be added
        // to a variety of memory locations.
        // First, the node is stored in a list of nodes specific to a grid
        // cell. If this is the first node in that grid cell, the list of 
        // occupied cells is updated. Then, the node is stored in a general
        // list that keeps track of all nodes for building the final
        // trajectory.

        void AddNode(Node n)
        {
            int cellNumber = GetCellNumber(n.x, n.y);
            if (numNodesInCell[cellNumber] < 1)
            {
                occupiedCellsList[numOccupiedCells] = cellNumber;
                numOccupiedCells++;
            }

            if (numNodesInCell[cellNumber] < 400)
            {
                NodesInCells[cellNumber, numNodesInCell[cellNumber]] = n;
                numNodesInCell[cellNumber]++;

                // Add to nodelist
                nodeList[numNodes] = n;
                numNodes++;
            }
            return;
        }


        // Given the goal node, this function will recursively add the
        // parent node to a trajectory until the start node is reached.
        // The result is a list of nodes that connect the start node to
        // the goal node with collision free edges.

        void BuildTraj(Node goalNode)
        {
            Node[] tempList = new Node[maxNumNodes];
            for (int j = 0; j < maxNumNodes; j++)
                trajList[j] = new Node(0, 0, 0, 0);

            tempList[0] = goalNode;
            int i = 1;

            // Make backwards traj by looking at parent of every child node
            while (tempList[i - 1].nodeIndex != 0)
            {
                tempList[i] = nodeList[tempList[i - 1].lastNode];
                i++;
            }

            // Reverse trajectory order
            for (int j = 0; j < i; j++)
            {
                trajList[j] = tempList[i - j - 1];
            }

            // Set size of trajectory and initialize node counter
            trajSize = i;
            trajCurrentNode = 0;

            return;
        }


 


        #endregion


        #region Localization Functions
        /************************ LOCALIZATION ***********************/

        // This function will grab the most recent encoder measurements
        // from either the simulator or the robot (whichever is activated)
        // and use those measurements to predict the RELATIVE forward 
        // motion and rotation of the robot. These are referred to as
        // distanceTravelled and angleTravelled respectively.
        public void MotionPrediction()
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

            // pulsesPerRotation is encoder resolution of 190 counts/rev
            // Distance traveled by each wheel (since the encoders have 190 counts/rev)
            wheelDistanceL = 2 * Math.PI * wheelRadius * (diffEncoderPulseL / (double)pulsesPerRotation);
            wheelDistanceR = 2 * Math.PI * wheelRadius * (diffEncoderPulseR / (double)pulsesPerRotation);

            // Average distance traveled by each wheel to get distance traveled by robot
            distanceTravelled = (wheelDistanceL + wheelDistanceR) / 2;

            // Angle traveled is difference in distances traveled by each wheel / 2*L
            // Angle traveled should be within -pi and pi
            angleTravelled = -(wheelDistanceL - wheelDistanceR) / (2 * robotRadius);

            // Calculate estimated states x_est, y_est, t_test
            x = x + distanceTravelled * Math.Cos(t + angleTravelled / 2);
            y = y + distanceTravelled * Math.Sin(t + angleTravelled / 2);
            t = t + angleTravelled;

            // Keep angle of robot within -pi and pi
            if (t >= Math.PI) // if angle is over pi
            {
                t = (t % Math.PI) - Math.PI; //roll over to -pi to 0 range
            }
            if (t <= -Math.PI) // if angle is less than pi
            {
                t = (t % Math.PI) + Math.PI; //roll over to 0 to pi range
            }

            // Update last encoder count variables
            lastEncoderPulseL = currentEncoderPulseL;
            lastEncoderPulseR = currentEncoderPulseR;

            // ****************** Additional Student Code: End   ************
        }

        // This function will Localize the robot, i.e. set the robot position
        // defined by x,y,t using the last position with angleTravelled and
        // distance travelled.
        public void LocalizeRealWithOdometry()
        {
            // ****************** Additional Student Code: Start ************

            // Put code here to calculate x,y,t based on odemetry 
            // (i.e. using last x, y, t as well as angleTravelled and distanceTravelled).
            // Make sure t stays between pi and -pi
            // Update the actual
            // Lab 2 Code

            x = x_est;
            y = y_est;
            t = t_est;
    
            // ****************** Additional Student Code: End   ************
        }
        // This function will Localize the robot, i.e. set the robot position
        // defined by x,y,t using the last position with angleTravelled and
        // distance travelled.
        public void LocalizeRealWithIMU()
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
            

            // ****************** Additional Student Code: Start ************

            // Put code here to calculate x_est, y_est, t_est using a PF

            // Edited in Lab 4

            double maxWeight = 0; // this is for the prediction step; sum of all particles' weights

            /// Code from Lab 2: Odometry 

            // Loop and propagate all particles
            // QUESTION: DO WE ADD RANDOMNESS OR ASSUME RANDOMNESS AS WE PROPAGATE?

            // Calculate stdev for each encoder value before looping through particles
            // Change this coef. to change the spread. Experimental value from lab 2 with infinte nb of particle was 0.14
            double stdevL = 0.05 * wheelDistanceL;
            double stdevR = 0.05 * wheelDistanceR;

            // Don't try to localize if the robot is not moving
            if (distanceTravelled == 0 && angleTravelled == 0) return;

            for (int i = 0; i < numParticles; ++i)
            {
                //PREDICTION STEP

                // 1) Calculate estimated states x_est, y_est, t_test based on odometry

                // Distance traveled by each wheel (since the encoders have 190 counts/rev)
                double randDistanceL = wheelDistanceL + RandomGaussian(0, stdevL);
                double randDistanceR = wheelDistanceR + RandomGaussian(0, stdevR);

                // Average distance traveled by each wheel to get distance traveled by robot
                double randDistanceTravelled = (randDistanceL + randDistanceR) / 2;

                // Angle traveled is difference in distances traveled by each wheel / 2*L
                // Angle traveled should be within -pi and pi
                double randAngleTravelled = -(randDistanceL - randDistanceR) / (2 * robotRadius);

                propagatedParticles[i].x = particles[i].x + randDistanceTravelled * Math.Cos(particles[i].t + randAngleTravelled / 2);
                propagatedParticles[i].y = particles[i].y + randDistanceTravelled * Math.Sin(particles[i].t + randAngleTravelled / 2);
                propagatedParticles[i].t = particles[i].t + randAngleTravelled;

                // 2) Calculate weight of particle
                propagatedParticles[i].w = CalculateWeight(i);
                maxWeight = Math.Max(maxWeight, propagatedParticles[i].w);

                // 3) Update set of particles with propagated particles
                //particles[i] = propagatedParticles[i].copy();  // WHY IS THAT HERE???? Does not make sense to me

            }

            // CORRECTION STEP
            if (newLaserData)
            {
                // new array to hold temporary particles
                // maximum size of tempParticles should be 4*numParticles
                Particle[] tempParticles = new Particle[4 * numParticles];

                // number of temporary particles
                int numTempParticles = 0;

                // Check that we have new measurements before resampling
                //if (distanceTravelled == 0 && angleTravelled == 0) return;

                // Resample particle using the approximate method
                //if (distanceTravelled != 0 || angleTravelled != 0)
                //{
                for (int i = 0; i < numParticles; ++i)
                {
                    // normalize weights so we can compare them on a range of 0 to 1.0
                    // set weight to 0 when the weight of all particles if 0
                    double weight = (maxWeight > 0) ? propagatedParticles[i].w / maxWeight : 0;

                    if (weight < 0.25) // add 1 copy
                    {
                        // add 1st copy
                        tempParticles[numTempParticles] = propagatedParticles[i].copy();
                        ++numTempParticles;
                    }
                    else if (weight < 0.5) // add 2 copy
                    {
                        // add 1st copy
                        tempParticles[numTempParticles] = propagatedParticles[i].copy();
                        ++numTempParticles;
                        // add 2nd copy
                        tempParticles[numTempParticles] = propagatedParticles[i].copy();
                        ++numTempParticles;
                    }
                    else if (weight < 0.75) // add 3 copy
                    {
                        // add 1st copy
                        tempParticles[numTempParticles] = propagatedParticles[i].copy();
                        ++numTempParticles;
                        // add 2nd copy
                        tempParticles[numTempParticles] = propagatedParticles[i].copy();
                        ++numTempParticles;
                        // add 3rd copy
                        tempParticles[numTempParticles] = propagatedParticles[i].copy();
                        ++numTempParticles;
                    }
                    else if (weight <= 1.0) // add 4 copies
                    {
                        // add 1st copy
                        tempParticles[numTempParticles] = propagatedParticles[i].copy();
                        ++numTempParticles;
                        // add 2nd copy
                        tempParticles[numTempParticles] = propagatedParticles[i].copy();
                        ++numTempParticles;
                        // add 3rd copy
                        tempParticles[numTempParticles] = propagatedParticles[i].copy();
                        ++numTempParticles;
                        // add 4th copy
                        tempParticles[numTempParticles] = propagatedParticles[i].copy();
                        ++numTempParticles;
                    }
                }

                // loop through the number of particles to resample from temporary particle list
                for (int i = 0; i < numParticles; ++i)
                {
                    // Note: fabiha is worried RandomGaussian() won't have enough resolution
                    // tried to make RandomGaussian() go from 0 to 1, instead of -2 to 2
                    // Guillaume wants to try with uniform
                    int r = random.Next(0, numTempParticles - 1);
                    particles[i] = tempParticles[r].copy();

                    newLaserData = false;
                }
            }
            else
            {
                for (int i = 0; i < numParticles; ++i)
                {
                    particles[i] = propagatedParticles[i].copy();
                }
            }

            double x_est_tot = 0;
            double y_est_tot = 0;
            double t_est_tot = 0;

            // Sum all particle state values
            double totalWeight = 0;
            for (int i = 0; i < numParticles; ++i)
            {
                x_est_tot = (particles[i].x) * particles[i].w + x_est_tot;
                y_est_tot = (particles[i].y) * particles[i].w + y_est_tot;
                t_est_tot = (particles[i].t) * particles[i].w + t_est_tot;
                totalWeight += particles[i].w;
            }

            // Average all particle state values to get x_est, y_est, and t_est
            if (totalWeight != 0)
            {
                x_est = x_est_tot / totalWeight;
                y_est = y_est_tot / totalWeight;
                t_est = t_est_tot / totalWeight;
            }

            //varxx = 0;
            //varyy = 0;
            //varxy = 0;
            //if (numParticles != 0)
            //{// Find standard deviation of particles
            //    for (int i = 0; i < numParticles; ++i)
            //    {
            //        varxx += Math.Pow(particles[i].x - x_est, 2); // sum of (x_particle - x_real) ^ 2
            //        varyy += Math.Pow(particles[i].y - y_est, 2); // sum of (y_particle - y_real) ^ 2
            //        varxy += (particles[i].y - y_est) * (particles[i].x - x_est);
            //    }
            //    varxx = varxx / (numParticles + 0.01); // average to find actual variance x
            //    varyy = varyy / (numParticles + 0.01); // average to find actual variance y
            //    varxy = varxy / (numParticles + 0.01);
            //}


            // ****************** Additional Student Code: End   ************

        }

        // Particle filters work by setting the weight associated with each
        // particle, according to the difference between the real robot 
        // range measurements and the predicted measurements associated 
        // with the particle.
        // This function should calculate the weight associated with particle p.

        void CalculateWeight(int p)
        {

            double weight = 1;

            // ****************** Additional Student Code: Start ************

            // Put code here to calculated weight. Feel free to use the
            // function map.GetClosestWallDistance from Map.cs.

            double particleDist, robotDist;
            for (int i = 0; i < LaserData.Length; i = i + laserStepSize * 3)
            {
                // particleDist is the distance from the particle to the closest wall
                particleDist = map.GetClosestWallDistance(propagatedParticles[p].x, propagatedParticles[p].y, propagatedParticles[p].t - 1.57 + laserAngles[i]);
                // robotDist is the distance from the robot to the closest wall
                robotDist = LaserData[i] / 1000.0;
                if (robotDist == 6.00 || particleDist == 6.00 || LaserData[i] < 100) weight *= 0.01;
                else
                {
                    double value = GaussianFunction(particleDist, robotDist, 0.2);
                    weight *= value;
                }
            }

            return weight;

            // ****************** Additional Student Code: End   ************

        }



        // This function is used to initialize the particle states 
        // for particle filtering. It should pick a random location in the 
        // environment for each particle by calling SetRandomPos

        void InitializeParticles() {


	        // Set particles in random locations and orientations within environment
	        for (int i=0; i< numParticles; i++){

		        // Either set the particles at known start position [0 0 0],  
		        // or set particles at random locations.

                if (jaguarControl.startMode == jaguarControl.UNKNOWN)
    		        SetRandomPos(i);
                else if (jaguarControl.startMode == jaguarControl.KNOWN)
		            SetStartPos(i);
	        }
            
        }



        // For particle p, this function will select a valid position. It should
        // select the position randomly, with equal likelihood of being anywhere 
        // in the environement. Should work for rectangular environments to make 
        // things easier.

        void SetRandomPos(int p){

	        // ****************** Additional Student Code: Start ************

	        // Put code here to calculated the position, orientation of 
            // particles[p]. Feel free to use the random.NextDouble() function. 
	        // It might be helpful to use boundaries defined in the
	        // Map.cs file (e.g. map.minX)

            particles[p].x = (random.Next((int)(numParticles * map.minX), (int)(numParticles * map.maxX))) / (double)numParticles;
            particles[p].y = (random.Next((int)(numParticles * map.minY), (int)(numParticles * map.maxY))) / (double)numParticles;
            particles[p].t = (random.Next((int)(numParticles * -1 * Math.PI), (int)(numParticles * 2 * Math.PI))) / numParticles;

            // ****************** Additional Student Code: End   ************
        }




        // For particle p, this function will select a start predefined position. 
        void SetStartPos(int p){
	        particles[p].x = initialX;
	        particles[p].y = initialY;
	        particles[p].t = initialT;
        }



        // Random number generator with gaussian distribution
        // Often random guassian numbers are used in particle filters. This
        // function might help.

        double RandomGaussian()
        {
	        double U1, U2, V1=0, V2;
	        double S = 2.0;
	        while(S >= 1.0) 
	        {
		        U1 = random.NextDouble();
                U2 = random.NextDouble();
		        V1 = 2.0*U1-1.0;
		        V2 = 2.0*U2-1.0;
		        S = Math.Pow(V1,2) + Math.Pow(V2,2);
	        }
	        double gauss = V1*Math.Sqrt((-2.0*Math.Log(S))/S);
	        return gauss;
        }


        // LAB 4 EDITED FUNCTION
        // Random number generator with gaussian distribution and given mean & stdev
        // Often random guassian numbers are used in particle filters. This
        // function might help.

        double RandomGaussian(double mean, double stdev)
        {
            return mean + stdev * RandomGaussian();
        }

        double GaussianFunction(double actual, double mean, double stdev)
        {
            //gauss = Math.Exp( - Math.Pow(actual - mean, 2) / (2 * Math.Pow(stdev, 2)));
            //peak = 1/ (Math.Sqrt(2*Math.PI));
            //gauss = guass / peak;
            return Math.Exp(-Math.Pow(actual - mean, 2) / (2 * Math.Pow(stdev, 2))) / (Math.Sqrt(2 * Math.PI) * stdev);
        }


        // Get the sign of a number
        double Sgn(double a)
        {
	        if (a>0)
                return 1.0;
	        else if (a<0)
                return -1.0;
	        else
                return 0.0;
        }

        #endregion

    }
}
