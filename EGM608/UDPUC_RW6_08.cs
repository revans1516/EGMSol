using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using abb.egm;
using System.Runtime.CompilerServices;

namespace EGM_6_08
{
	public class UDPUC_RW6_08
	{


	/// <summary>
	/// These are the different motion types egm can use
	/// </summary>
		public enum MotionType
		{
			Eular,
			Quaternion,
			Joint,
		};

		public struct Position
		{
			public double[] Pos;
			public double[] Eular;
			public double[] Quat;
			public double[] Joints;
			public Position(double[] _Pos, double[] _Eular, double[] _Quat, double[] _Joints)
			{
				Pos = _Pos;
				Eular=_Eular;
				Quat=_Quat;
				Joints=_Joints;
			}
			
		}

		public MotionType Move_Type = MotionType.Joint;
		private EgmHeader.Types.MessageType EGMtype;
		private Thread _SensorThread = null;
		private UdpClient _UDPServer = null;
		private int UDPPort;
		private bool _RunThread = true;
		private uint _SeqNumber = 0;
		private double[] PoseEular = { 0, 0, 0, 0, 0, 0 };
		private double[] Joints = { 0, 0, 0, 0, 0, 0 };
		private double[] PoseQuat = { 0, 0, 0, 1, 0, 0, 0 };
		public Position CurrentPos= new Position ( new double[] { 0, 0, 0 }, new double[] {0,0, 0 }, new double[] {1,0,0, 0}, new double[]{0,0,0,0,0,0} );


		/// <summary>
		/// Creates an instance of the egm communication.
		/// </summary>
		/// <param name="UDPPortNum">The port the robot is configured to listen on</param>
		/// <param name="Move">The motion type to be used with EGM</param>
		public UDPUC_RW6_08(int UDPPortNum, MotionType Move,EgmHeader.Types.MessageType messageType)
		{
			UDPPort = UDPPortNum;
			Move_Type = Move;
			EGMtype = messageType;
		}

		/// <summary>
		/// the Values to be sent to EGM in XYZ Eular ZYX
		/// </summary>
		/// <param name="X">The X value to send to egm</param>
		/// <param name="Y">The Y value to be sent to egm</param>
		/// <param name="Z">The Y value to be sent to egm</param>
		/// <param name="RX">The Roll</param>
		/// <param name="RY">The Pitch</param>
		/// <param name="RZ">The Yaw</param>
		public void SetEularPose(double X, double Y, double Z, double RX, double RY, double RZ)
		{
			PoseEular = new double[] { X, Y, Z, RX, RY, RZ };
		}


		/// <summary>
		/// Values sent to egm as xyz quaternion wxyz format
		/// </summary>
		/// <param name="X">x translation</param>
		/// <param name="Y">y translation</param>
		/// <param name="Z">z translation</param>
		/// <param name="RW">W value of quaternion</param>
		/// <param name="RX">X value of quaternion</param>
		/// <param name="RY">Y value of quaternion</param>
		/// <param name="RZ">Z value of quaternion</param>
		public void SetOrientPose(double X, double Y, double Z, double RW, double RX, double RY, double RZ)
		{
			PoseQuat = new double[] { X, Y, Z, RW, RX, RY, RZ };
		}


		/// <summary>
		/// Joint values the robot is meant to be sent to
		/// </summary>
		/// <param name="J1">value to joint 1</param>
		/// <param name="J2">value to joint 1</param>
		/// <param name="J3">value to joint 1</param>
		/// <param name="J4">value to joint 1</param>
		/// <param name="J5">value to joint 1</param>
		/// <param name="J6">value to joint 1</param>
		public void SetJoint(double J1, double J2, double J3, double J4, double J5, double J6)
		{
			Joints = new double[] { J1, J2, J3, J4, J5, J6 };
		}

		private void UDPUCPoseThread()
		{

			_UDPServer = new UdpClient(UDPPort);

			var remoteEP = new IPEndPoint(IPAddress.Any, UDPPort);

			while (_RunThread)
			{
				var data = _UDPServer.Receive(ref remoteEP);
				if (data != null)
				{
					GetCurrentPos(data);
					EgmSensor.Builder SensorData = EgmSensor.CreateBuilder();

					switch (Move_Type)
					{
						case MotionType.Eular:
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
						EgmSensor SensorMessage = SensorData.Build();
						SensorMessage.WriteTo(MemStream);

						int bytesSent = _UDPServer.Send(MemStream.ToArray(), (int)MemStream.Length, remoteEP);
					}

				}
			}
		}

		private void CreateEularMessage(EgmSensor.Builder sensor)
		{
			// create a header
			EgmHeader.Builder hdr = new EgmHeader.Builder();
			hdr.SetSeqno(_SeqNumber++)
				.SetTm((uint)DateTime.Now.Ticks)
				.SetMtype(EGMtype);

			sensor.SetHeader(hdr);

			// create some sensor data
			EgmPlanned.Builder planned = new EgmPlanned.Builder();
			EgmPose.Builder pos = new EgmPose.Builder();
			EgmEuler.Builder pe = new EgmEuler.Builder();
			EgmCartesian.Builder pc = new EgmCartesian.Builder();

			pc.SetX(PoseEular[0])
				.SetY(PoseEular[1])
				.SetZ(PoseEular[2]);

			pe.SetX(PoseEular[3])
			   .SetY(PoseEular[4])
			   .SetZ(PoseEular[5]);

			pos.SetPos(pc)
				.SetEuler(pe);

			planned.SetCartesian(pos);  // bind pos object to planned
			sensor.SetPlanned(planned); // bind planned to sensor object

			return;
		}

		private void CreateQuatMessage(EgmSensor.Builder sensor)
		{
			// create a header
			EgmHeader.Builder hdr = new EgmHeader.Builder();
			hdr.SetSeqno(_SeqNumber++)
				.SetTm((uint)DateTime.Now.Ticks)
				.SetMtype(EGMtype);

			sensor.SetHeader(hdr);

			// create some sensor data
			EgmPlanned.Builder planned = new EgmPlanned.Builder();
			EgmPose.Builder pos = new EgmPose.Builder();
			EgmQuaternion.Builder pq = new EgmQuaternion.Builder();
			EgmCartesian.Builder pc = new EgmCartesian.Builder();

			pc.SetX(PoseQuat[0])
				.SetY(PoseQuat[1])
				.SetZ(PoseQuat[2]);

			pq.SetU0(PoseQuat[3])
				.SetU1(PoseQuat[4])
				.SetU2(PoseQuat[5])
				.SetU3(PoseQuat[6]);


			pos.SetPos(pc)
				.SetOrient(pq);

			planned.SetCartesian(pos);  // bind pos object to planned
			sensor.SetPlanned(planned); // bind planned to sensor object

			return;
		}

		private void CreateJointMessage(EgmSensor.Builder sensor)
		{
			// create a header
			EgmHeader.Builder hdr = new EgmHeader.Builder();
			hdr.SetSeqno(_SeqNumber++)
				.SetTm((uint)DateTime.Now.Ticks)
				.SetMtype(EgmHeader.Types.MessageType.MSGTYPE_CORRECTION);

			sensor.SetHeader(hdr);

			// create some sensor data
			EgmPlanned.Builder planned = new EgmPlanned.Builder();
			EgmJoints.Builder joint = new EgmJoints.Builder();

			joint.AddJoints(1);
			joint.AddJoints(2);
			joint.AddJoints(3);
			joint.AddJoints(4);
			joint.AddJoints(5);
			joint.AddJoints(6);

			joint.SetJoints(0, Joints[0])
				  .SetJoints(1, Joints[1])
				  .SetJoints(2, Joints[2])
				  .SetJoints(3, Joints[3])
				  .SetJoints(4, Joints[4])
				  .SetJoints(5, Joints[5]);

			planned.SetJoints(joint);
			sensor.SetPlanned(planned); // bind planned to sensor object

			return;
		}

		private void DisplayInboundMessage(EgmRobot robot)
		{
			if (robot.HasHeader && robot.Header.HasSeqno && robot.Header.HasTm)
			{
				//Console.WriteLine("Seq={0} tm={1}",
				//robot.Header.Seqno.ToString(), robot.Header.Tm.ToString());
				Console.WriteLine("Seq={0} tm={1}",
					robot.FeedBack.Cartesian.Pos.X, robot.Header.Tm.ToString());
			}
			else
			{
				Console.WriteLine("No header in robot message");
			}
		}

		private void GetCurrentPos(byte[] data)
		{
			EgmRobot robot = EgmRobot.CreateBuilder().MergeFrom(data).Build();
			CurrentPos.Pos = new double[] {robot.FeedBack.Cartesian.Pos.X,
			robot.FeedBack.Cartesian.Pos.Y,
			robot.FeedBack.Cartesian.Pos.Z, };
			CurrentPos.Quat= new double[]{robot.FeedBack.Cartesian.Orient.U0,
			robot.FeedBack.Cartesian.Orient.U1,
			robot.FeedBack.Cartesian.Orient.U2,
			robot.FeedBack.Cartesian.Orient.U3};
			CurrentPos.Eular = new double[] { robot.FeedBack.Cartesian.Euler.X,
			robot.FeedBack.Cartesian.Euler.Y,
			robot.FeedBack.Cartesian.Euler.Z};
			CurrentPos.Joints = robot.FeedBack.Joints.JointsList.ToArray();
		}

		/// <summary>
		/// Starts the commnication thread with the robot
		/// </summary>
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

		/// <summary>
		/// Stops communcation
		/// </summary>
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