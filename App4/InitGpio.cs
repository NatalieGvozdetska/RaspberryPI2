//Class for initialize GPIOs and get parameter "value" for metod PulseIn

using System;
using System.Diagnostics;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using Windows.Devices.Enumeration;
using Windows.Devices.Gpio;
using Windows.Devices.SerialCommunication;
using Windows.Devices.Spi;

namespace App4
{
    class InitGpio
    {
        //Initialize all variable
        public static int Rele_PIN = 26;// rele on of
        public static int Bluetooth_PIN = 16;//pin for indicating bluetooth connection
        public static GpioPin _relePin;
        public static GpioPin _BluetoothPin;

        //Method for initialize
        public static void Init()
        {
            var gpio = GpioController.GetDefault();
            _relePin = gpio.OpenPin(Rele_PIN);
            _BluetoothPin = gpio.OpenPin(Bluetooth_PIN);
            _relePin.SetDriveMode(GpioPinDriveMode.Output);
            _BluetoothPin.SetDriveMode(GpioPinDriveMode.Output);
            _relePin.Write(GpioPinValue.Low);//Low or High
            _BluetoothPin.Write(GpioPinValue.High);
            Bluetooth.InitializeRfcommDeviceService();

        }

    }
}
