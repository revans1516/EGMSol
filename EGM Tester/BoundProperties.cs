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