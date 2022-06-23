using System;
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
		static EGM_6_10.UDPUC_RW6_10 TestingSender;
		static public double Sliderx { get; set; }
		static public double Slidery { get; set; }
		static public double Sliderz { get; set; }
		static public double Sliderrx { get; set; }
		static public double Sliderry { get; set; }
		static public double Sliderrz { get; set; }
		BoundProperties bp;
		public MainWindow()
		{
			InitializeComponent();
			bp = new BoundProperties();
			this.DataContext = bp;
			Testing_Thread = new System.Threading.Thread(EGMSender);
		}

		private void btnStartSend_Click(object sender, RoutedEventArgs e)
		{
			Testing_Thread.Start();
		}

		private void EGMSender()
		{
			
			TestingSender = new EGM_6_10.UDPUC_RW6_10(6510,EGM_6_10.UDPUC_RW6_10.MotionType.Euler);

			TestingSender.Start();

			while(true)
			{
				
				TestingSender.SetEularPose(bp.X, bp.Y, bp.Z, bp.RX, bp.RY, bp.RZ);

				System.Threading.Thread.Sleep(4);
			}

		}


	}
}
