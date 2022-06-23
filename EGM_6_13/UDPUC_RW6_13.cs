
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using System.Threading.Tasks;
using Abb.Egm;


namespace EGM_6_13
{

    public class UDPUC_RW6_13
    {
        public enum MotionType
        {
            Euler,
            Quaternion,
            Joint,
        };
        public enum GuidanceType
        {
            Guidance = 3,
            Correction = 4
        }

        public MotionType Move_Type = MotionType.Joint;
        private Thread _SensorThread = null;
        private UdpClient _UDPServer = null;
        private int UDPPort;
        private bool _RunThread = true;
        private uint _SeqNumber = 0;
        private double[] PoseEular = { 0, 0, 0, 0, 0, 0 };
        private double[] Joints = { 0, 0, 0, 0, 0, 0 };
        private double[] PoseQuat = { 0, 0, 0, 1, 0, 0, 0 };


        public double[] CurrentPose = { 0, 0, 0, 1, 0, 0, 0 };


        public UDPUC_RW6_13(int UDPPortNum, MotionType Move)
        {
            UDPPort = UDPPortNum;
            Move_Type = Move;
        }


        public UInt64 RobotTime
        {
            get { return RobotTime; }

            private set { }
        }


        public void SetEularPose(double X, double Y, double Z, double RX, double RY, double RZ)
        {
            PoseEular = new double[] { X, Y, Z, RX, RY, RZ };
        }

        public void SetOrientPose(double X, double Y, double Z, double RW, double RX, double RY, double RZ)
        {
            PoseQuat = new double[] { X, Y, Z, RW, RX, RY, RZ };
        }

        public void SetJoint(double J1, double J2, double J3, double J4, double J5, double J6)
        {
            Joints = new double[] { J1, J2, J3, J4, J5, J6 };
        }



        // Display message from robot
        void DisplayInboundMessage(EgmRobot robot)
        {
            if (robot.Header != null && robot.Header.HasSeqno && robot.Header.HasTm)
            {
                Console.WriteLine("Seq={0} tm={1}",
                    robot.Header.Seqno.ToString(), robot.Header.Tm.ToString());
            }
            else
            {
                Console.WriteLine("No header in robot message");
            }
        }

        //////////////////////////////////////////////////////////////////////////
        // Create a sensor message to send to the robot
        void CreateQuatMessage(EgmSensor sensor, double OptLinSpeed = -1, double OptOrientSpeed = -1)
        {
            // create a header
            EgmHeader hdr = new EgmHeader();
            // EgmSpeedRef speedbuilder = new EgmSpeedRef();
            hdr.Seqno = _SeqNumber++;
            hdr.Tm = (uint)DateTime.Now.Ticks;
            hdr.Mtype = EgmHeader.Types.MessageType.MsgtypeCorrection;

            sensor.Header = hdr;

            // create some sensor data
            EgmPlanned planned = new EgmPlanned();
            EgmPose pos = new EgmPose();
            EgmQuaternion pq = new EgmQuaternion();
            EgmCartesian pc = new EgmCartesian();

            pc.X = PoseQuat[0];
            pc.Y = PoseQuat[1];
            pc.Z = PoseQuat[2];

            pq.U0 = PoseQuat[3];
            pq.U1 = PoseQuat[4];
            pq.U2 = PoseQuat[5];
            pq.U3 = PoseQuat[6];

            pos.Pos = pc;
            pos.Orient = pq;

            planned.Cartesian = pos;  // bind pos object to planned
            sensor.Planned = planned; // bind planned to sensor object

            //if (OptLinSpeed > 0 && OptOrientSpeed > 0) sensor.SpeedRef= speedbuilder;

            return;
        }



        private void CreateEularMessage(EgmSensor sensor, double OptLinSpeed = -1, double OptOrientSpeed = -1)
        {
            // create a header
            EgmHeader hdr = new EgmHeader();
            EgmSpeedRef speedbuilder = new EgmSpeedRef();
            EgmCartesianSpeed CartSpeed = new EgmCartesianSpeed();


            hdr.Seqno = _SeqNumber++;
            hdr.Tm = (uint)DateTime.Now.Ticks;
            hdr.Mtype = EgmHeader.Types.MessageType.MsgtypeCorrection;

            sensor.Header = hdr;

            // create some sensor data
            EgmPlanned planned = new EgmPlanned();
            EgmPose pos = new EgmPose();
            EgmEuler pe = new EgmEuler();
            EgmCartesian pc = new EgmCartesian();

            pc.X = PoseQuat[0];
            pc.Y = PoseQuat[1];
            pc.Z = PoseQuat[2];

            pe.X = (PoseEular[3]);
            pe.Y = (PoseEular[4]);
            pe.Z = (PoseEular[5]);

            pos.Pos = pc;
            pos.Euler = pe;

            planned.Cartesian = pos;  // bind pos object to planned
            sensor.Planned = planned; // bind planned to sensor object

            //if (OptLinSpeed > 0 && OptOrientSpeed > 0) sensor.SetSpeedRef(speedbuilder);

            return;
        }


        private void CreateJointMessage(EgmSensor sensor)
        {
            // create a header
            EgmHeader hdr = new EgmHeader();
            hdr.Seqno = _SeqNumber++;
            hdr.Tm = (uint)DateTime.Now.Ticks;
            hdr.Mtype = EgmHeader.Types.MessageType.MsgtypeCorrection;

            sensor.Header = hdr;

            // create some sensor data
            EgmPlanned planned = new EgmPlanned();
            EgmJoints joint = new EgmJoints();

            joint.Joints.Add(1);
            joint.Joints.Add(2);
            joint.Joints.Add(3);
            joint.Joints.Add(4);
            joint.Joints.Add(5);
            joint.Joints.Add(6);

            joint.Joints[0] = Joints[0];
            joint.Joints[1] = Joints[1];
            joint.Joints[2] = Joints[2];
            joint.Joints[3] = Joints[3];
            joint.Joints[4] = Joints[4];
            joint.Joints[5] = Joints[5];


            planned.Joints = (joint);
            sensor.Planned = (planned); // bind planned to sensor object

            return;
        }



        private void UDPUCPoseThread()
        {

            _UDPServer = new UdpClient(UDPPort);

            var remoteEP = new IPEndPoint(IPAddress.Any, UDPPort);

            while (_RunThread)
            {
                var data = _UDPServer.Receive(ref remoteEP);
                System.Diagnostics.Debug.WriteLine(data.Length);
                if (data != null)
                {
                    GetCurrentPos(data);
                    EgmSensor SensorData = new EgmSensor();

                    switch (Move_Type)
                    {
                        case MotionType.Euler:
                            CreateEularMessage(SensorData);
                            break;
                        case MotionType.Quaternion:
                            CreateQuatMessage(SensorData);
                            break;
                        case MotionType.Joint:
                            CreateJointMessage(SensorData);
                            break;
                    }

                    using (MemoryStream MemStream = new MemoryStream())
                    {
                        EgmSensor SensorMessage = SensorData;
                        int messageSize = SensorMessage.CalculateSize();
                        Google.Protobuf.CodedOutputStream codedOutputStream = new Google.Protobuf.CodedOutputStream(MemStream, messageSize);
                        SensorMessage.WriteTo(codedOutputStream);

                        int bytesSent = _UDPServer.Send(MemStream.ToArray(), (int)MemStream.Length, remoteEP);

                    }

                }
            }
        }



        private void GetCurrentPos(byte[] data)
        {
            EgmRobot robot = EgmRobot.Parser.ParseFrom(data);
            CurrentPose = new double[] {robot.FeedBack.Cartesian.Pos.X,
            robot.FeedBack.Cartesian.Pos.Y,
            robot.FeedBack.Cartesian.Pos.Z,
            robot.FeedBack.Cartesian.Orient.U0,
            robot.FeedBack.Cartesian.Orient.U1,
            robot.FeedBack.Cartesian.Orient.U2,
            robot.FeedBack.Cartesian.Orient.U3};
            RobotTime = robot.FeedBack.Time.Usec;


        }


        // Start a thread to listen on inbound messages
        public void Start()
        {
            if (_SensorThread != null && _SensorThread.IsAlive)
            {
                _SensorThread.Abort();
            }
            _RunThread = true;
            _SensorThread = new Thread(new ThreadStart(UDPUCPoseThread));
            _SensorThread.Start();
        }

        // Stop and exit thread
        public void Stop()
        {
            _RunThread = false;
            if (_SensorThread != null)
            {
                _SensorThread.Interrupt();
                _SensorThread.Abort();
            }
            if (_UDPServer != null)
            {
                _UDPServer.Close();
            }

        }
    }
}


