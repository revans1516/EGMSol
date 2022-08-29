using System;
using System.Numerics;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;

namespace EGM_Tester
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        System.Threading.Thread Testing_Thread;
        System.Threading.Thread Lib_Thread;
        System.Threading.Thread BackGroundThread;
        ABB.Robotics.Controllers.Controller YumiController;
        ABB.Robotics.Controllers.ControllerInfoCollection controllers;

        ABB.Robotics.Controllers.IOSystemDomain.DigitalSignal LeftGripGripper;
        ABB.Robotics.Controllers.IOSystemDomain.DigitalSignal RightGripGripper;
        ABB.Robotics.Controllers.IOSystemDomain.DigitalSignal StopEGM;

        Liberty.PlStream LibStream;

        static public Boolean RunLibStream = true;
        static public Boolean RunTestingStream = true;

        static EGM_6_10.UDPUC_RW6_10 TestingSenderR;
        static EGM_6_10.UDPUC_RW6_10 TestingSenderL;
        public System.Diagnostics.Process LibertyProcess;
        BoundProperties bp;


        public MainWindow()
        {
            InitializeComponent();
            bp = new BoundProperties();
            this.DataContext = bp;
            Testing_Thread = new System.Threading.Thread(EGMSender);
            Lib_Thread = new System.Threading.Thread(LibSender);
            BackGroundThread = new System.Threading.Thread(BackgroundProc);

            ABB.Robotics.Controllers.Discovery.NetworkScanner scanner = new ABB.Robotics.Controllers.Discovery.NetworkScanner();
            ABB.Robotics.Controllers.Discovery.NetworkScanner.AddRemoteController("192.168.125.1");

            scanner.Scan();


            controllers = scanner.Controllers;

            YumiController = ABB.Robotics.Controllers.Controller.Connect(controllers[0], ABB.Robotics.Controllers.ConnectionType.Standalone);
            LeftGripGripper = (ABB.Robotics.Controllers.IOSystemDomain.DigitalSignal)YumiController.IOSystem.GetSignal("doEGM_GripperL");
            RightGripGripper = (ABB.Robotics.Controllers.IOSystemDomain.DigitalSignal)YumiController.IOSystem.GetSignal("doEGM_GripperR");
            StopEGM = (ABB.Robotics.Controllers.IOSystemDomain.DigitalSignal)YumiController.IOSystem.GetSignal("doEGM_Stop");
            TestingSenderL = new EGM_6_10.UDPUC_RW6_10(6511, EGM_6_10.UDPUC_RW6_10.MotionType.Quaternion);
            TestingSenderR = new EGM_6_10.UDPUC_RW6_10(6510, EGM_6_10.UDPUC_RW6_10.MotionType.Quaternion);

            LibertyProcess = new System.Diagnostics.Process();
            LibertyProcess.StartInfo.FileName = System.Reflection.Assembly.GetEntryAssembly().Location.Replace("EGM Tester.exe", "") + "Liberty Interface/UnityExport.exe";
            LibertyProcess.StartInfo.CreateNoWindow = true;
            LibertyProcess.StartInfo.WindowStyle = System.Diagnostics.ProcessWindowStyle.Minimized;
            LibertyProcess.Start();

            LibStream = new Liberty.PlStream(Liberty.PlTracker.Liberty);
            LibStream.StartReading();

            System.Threading.Thread.Sleep(1000);
            BackGroundThread.Start();
        }

        private void BackgroundProc()
        {
            int TimeCounter = 0;

            while (true)
            {
                if (LibStream.digio[0] == 1 && LibStream.digio[1] == 1)
                {
                    TimeCounter++;
                }
                else
                {
                    TimeCounter = 0;
                }
                    

                if (LibStream.digio[0] == 1 && LibStream.digio[1] == 1 && TimeCounter>300 && Lib_Thread.IsAlive==false)
                {
                    StopEGM.Reset();
                    Lib_Thread = new System.Threading.Thread(LibSender);
                    Lib_Thread.Start();

                }
                LeftGripGripper.Value = LibStream.digio[0];
                RightGripGripper.Value = LibStream.digio[1];
                System.Threading.Thread.Sleep(10);
            }


        }

        private void btnStartSend_Click(object sender, RoutedEventArgs e)
        {
            Testing_Thread.Start();
        }

        private void btnStartLib_Click(object sender, RoutedEventArgs e)
        {
            Lib_Thread.Start();
        }


        private void LibSender()
        {
            int Loops = 0;
            int CycleTime = 4;

            System.Numerics.Vector3 StartLeftPos;
            System.Numerics.Vector3 StartRightPos;
            System.Numerics.Quaternion StartLeftOrient;
            System.Numerics.Quaternion StartRightOrient;

            System.Numerics.Matrix4x4 LeftMat;
            System.Numerics.Matrix4x4 RightMat;
            System.Numerics.Matrix4x4 InvertedLeftMat;
            System.Numerics.Matrix4x4 InvertedRightMat;

            System.Numerics.Vector3 LeftPos;
            System.Numerics.Quaternion LeftOrient;
            System.Numerics.Vector4 LeftVec4;
            System.Numerics.Vector3 RightPos;
            System.Numerics.Quaternion RightOrient;
            System.Numerics.Vector4 RightVec4;

            StopEGM.Reset();

            TestingSenderL.Start();
            TestingSenderR.Start();


            System.Threading.Thread.Sleep(3000);
            StartLeftPos = LibStream.positions[0];
            StartRightPos = LibStream.positions[1];

            StartLeftOrient = new System.Numerics.Quaternion(LibStream.orientations[0].X, LibStream.orientations[0].Y, LibStream.orientations[0].Z, LibStream.orientations[0].W);
            StartRightOrient = new System.Numerics.Quaternion(LibStream.orientations[1].X, LibStream.orientations[1].Y, LibStream.orientations[1].Z, LibStream.orientations[1].W);


            //StartLeftOrient = new System.Numerics.Quaternion(0,0,0,1);
            //StartRightOrient = new System.Numerics.Quaternion(0, 0, 0, 1);



            while (RunLibStream && Loops < necRuntime.Value / (CycleTime / 1000))
            {
                LeftPos = LibStream.positions[0] - StartLeftPos;
                RightPos = LibStream.positions[1] - StartRightPos;
                //LeftPos = new System.Numerics.Vector3(LeftPos.X, LeftPos.Y, LeftPos.Z);
                //RightPos = new System.Numerics.Vector3(RightPos.X, RightPos.Y, RightPos.Z);

                LeftVec4 = LibStream.orientations[0];
                RightVec4 = LibStream.orientations[1];

                LeftMat = System.Numerics.Matrix4x4.CreateFromQuaternion(new System.Numerics.Quaternion(LeftVec4.X, LeftVec4.Y, LeftVec4.Z, LeftVec4.W));
                RightMat = System.Numerics.Matrix4x4.CreateFromQuaternion(new System.Numerics.Quaternion(RightVec4.X, RightVec4.Y, RightVec4.Z, RightVec4.W));

                InvertedLeftMat = Matrix4x4.CreateFromQuaternion(StartLeftOrient);
                InvertedRightMat = Matrix4x4.CreateFromQuaternion(StartRightOrient);
                Matrix4x4.Invert(InvertedLeftMat, out InvertedLeftMat);
                Matrix4x4.Invert(InvertedRightMat, out InvertedRightMat);

                LeftOrient = System.Numerics.Quaternion.CreateFromRotationMatrix(System.Numerics.Matrix4x4.Multiply(LeftMat, InvertedLeftMat));
                RightOrient = System.Numerics.Quaternion.CreateFromRotationMatrix(System.Numerics.Matrix4x4.Multiply(RightMat, InvertedRightMat));

                LeftOrient = System.Numerics.Quaternion.Normalize(LeftOrient);
                RightOrient = System.Numerics.Quaternion.Normalize(RightOrient);

                LeftOrient = new System.Numerics.Quaternion(-LeftOrient.Z, LeftOrient.Y, -LeftOrient.X, LeftOrient.W);
                RightOrient = new System.Numerics.Quaternion(-RightOrient.Z, RightOrient.Y, -RightOrient.X, RightOrient.W);


                TestingSenderR.SetOrientPose(System.Numerics.Vector3.Multiply(RightPos, (float)25.4), RightOrient);
                TestingSenderL.SetOrientPose(System.Numerics.Vector3.Multiply(LeftPos, (float)25.4), LeftOrient);


                bp.CurrX = TestingSenderR.CurrentPose[0];
                bp.CurrY = TestingSenderR.CurrentPose[1];
                bp.CurrZ = TestingSenderR.CurrentPose[2];
                bp.CurrRX = TestingSenderR.CurrentPose[3];
                bp.CurrRY = TestingSenderR.CurrentPose[4];
                bp.CurrRZ = TestingSenderR.CurrentPose[5];

                bp.J1 = TestingSenderR.CurrentJoints[0];
                bp.J2 = TestingSenderR.CurrentJoints[1];
                bp.J3 = TestingSenderR.CurrentJoints[2];
                bp.J4 = TestingSenderR.CurrentJoints[3];
                bp.J5 = TestingSenderR.CurrentJoints[4];
                bp.J6 = TestingSenderR.CurrentJoints[5];

                System.Threading.Thread.Sleep(CycleTime);
                Loops++;
            }


            StopEGM.Set();

        }

        private void EGMSender()
        {


            TestingSenderR.Start();
            TestingSenderL.Start();

            while (RunTestingStream)
            {

                TestingSenderR.SetEularPose(bp.X, bp.Y, bp.Z, bp.RX, bp.RY, bp.RZ);
                TestingSenderL.SetEularPose(bp.X, bp.Y, bp.Z, bp.RX, bp.RY, bp.RZ);

                System.Threading.Thread.Sleep(4);
            }

        }

        private void Window_Closed(object sender, EventArgs e)
        {
            //if (Testing_Thread.IsAlive) Testing_Thread.Abort();
            //if (Lib_Thread.IsAlive) Lib_Thread.Abort();
            RunLibStream = false;
            RunTestingStream = false;

            if (LibStream != null) LibStream.StopPolhemus();
            if (LibertyProcess != null) LibertyProcess.CloseMainWindow();

            TestingSenderL.Stop();
            TestingSenderR.Stop();
            BackGroundThread.Abort();
        }
    }
}
