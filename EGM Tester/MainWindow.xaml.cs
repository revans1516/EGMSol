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
		System.Threading.Thread Testing_Thread = new System.Threading.Thread(EGMSender);
		static EGM_6_11.UDPUC_RW6_11 TestingSender;
		static public double Sliderx { get; set; }
		static public double Slidery { get; set; }
		static public double Sliderz { get; set; }
		static public double Sliderrx { get; set; }
		static public double Sliderry { get; set; }
		static public double Sliderrz { get; set; }

		public MainWindow()
		{
			InitializeComponent();
		}

		private void btnStartSend_Click(object sender, RoutedEventArgs e)
		{
			Testing_Thread.Start();
		}

		static private void EGMSender()
		{
			
			TestingSender = new EGM_6_11.UDPUC_RW6_11(6510,EGM_6_11.UDPUC_RW6_11.MotionType.Euler);

			TestingSender.Start();

			while(true)
			{
				
				TestingSender.SetEularPose(Sliderx, Slidery, Sliderz, Sliderrx, Sliderry, Sliderrz);

				System.Threading.Thread.Sleep(4);
			}

		}


	}
}
