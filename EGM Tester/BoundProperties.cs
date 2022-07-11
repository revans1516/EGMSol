using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Media.Media3D;
namespace EGM_Tester
{
    public class BoundProperties : INotifyPropertyChanged
    {
        private double _X;
        public double X
        {
            get { return _X; }
            set
            {
                _X = value;
                OnPropertyChanged();
            }
        }
        private double _Y;
        public double Y
        {
            get { return _Y; }
            set
            {
                _Y = value;
                OnPropertyChanged();
            }
        }
        private double _Z;
        public double Z
        {
            get { return _Z; }
            set
            {
                _Z = value;
                OnPropertyChanged();
            }
        }
        private double _RX;
        public double RX
        {
            get { return _RX; }
            set
            {
                _RX = value;
                OnPropertyChanged();
            }
        }
        private double _RY;
        public double RY
        {
            get { return _RY; }
            set
            {
                _RY = value;
                OnPropertyChanged();
            }
        }
        private double _RZ;
        public double RZ
        {
            get { return _RZ; }
            set
            {
                _RZ = value;
                OnPropertyChanged();
            }
        }


        private double _CurrX;
        public double CurrX
        {
            get { return _CurrX; }
            set
            {
                _CurrX = value;
                OnPropertyChanged();
            }
        }

        private double _CurrY;
        public double CurrY
        {
            get { return _CurrY; }
            set
            {
                _CurrY = value;
                OnPropertyChanged();
            }
        }

        private double _CurrZ;
        public double CurrZ
        {
            get { return _CurrZ; }
            set
            {
                _CurrZ = value;
                OnPropertyChanged();
            }
        }


        private double _CurrRX;
        public double CurrRX
        {
            get { return _CurrRX; }
            set
            {
                _CurrRX = value;
                OnPropertyChanged();
            }
        }

        private double _CurrRY;
        public double CurrRY
        {
            get { return _CurrRY; }
            set
            {
                _CurrRY = value;
                OnPropertyChanged();
            }
        }

        private double _CurrRZ;
        public double CurrRZ
        {
            get { return _CurrRZ; }
            set
            {
                _CurrRZ = value;
                OnPropertyChanged();
            }
        }

        private double _J1;
        public double J1
        {
            get { return _J1; }
            set
            {
                _J1 = value;
                OnPropertyChanged();
            }
        }

        private double _J2;
        public double J2
        {
            get { return _J2; }
            set
            {
                _J2 = value;
                OnPropertyChanged();
            }
        }

        private double _J3;
        public double J3
        {
            get { return _J3; }
            set
            {
                _J3 = value;
                OnPropertyChanged();
            }
        }

        private double _J4;
        public double J4
        {
            get { return _J4; }
            set
            {
                _J4 = value;
                OnPropertyChanged();
            }
        }

        private double _J5;
        public double J5
        {
            get { return _J5; }
            set
            {
                _J5 = value;
                OnPropertyChanged();
            }
        }

        private double _J6;
        public double J6
        {
            get { return _J6; }
            set
            {
                _J6 = value;
                OnPropertyChanged();
            }
        }

        public BoundProperties()
        {

        }

        // Delegates can be used in event handling to pass values to the UI thread.
        // To implement the INotifyPropertyChanged interface, you must register the PropertyChangedEventHandler delegate as an event.
        public event PropertyChangedEventHandler PropertyChanged;

        // When the PropertyChanged event is raised, this method will instantiate an object containing the name of the property that was changed 
        // so the UI control can connect to the appropriate property.
        protected void OnPropertyChanged([CallerMemberName] string propertyName = "")
        {
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));
        }
    }
}