using System;
using System.Diagnostics;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using Windows.Devices.Gpio;
using Windows.UI.Xaml;
using Windows.UI.Xaml.Navigation;
using Newtonsoft.Json;
using Microsoft.Azure.Devices.Client;
using Windows.Networking.Connectivity;
using System.Linq;
using System.Collections.Generic;
using Windows.Devices.Spi;
using Windows.Devices.Enumeration;
using System.Collections.ObjectModel;
using System.Runtime.InteropServices;
using System.IO;
using Windows.System;
using Windows.Storage.Streams;
using Windows.UI.Core;
using Windows.Foundation;


using Windows.UI.Xaml.Controls;
using Windows.Devices.SerialCommunication;

namespace App4
{
    public sealed partial class MainPage
    {
        private static Queue<object> _buffer;
        private static Task _receiveTask;
        private static Task _sendTask;
        private static Task _getSendingTask;
        public static CancellationTokenSource _receiveTokenSource;
        private static string DeviceID = "RaspberryData";
        private bool onof = false;
        private int i = 0;

        //private static OneWire onewire;

        public MainPage()
        {
            //Initiailizing pins
            InitGpio.Init();
            InitializeComponent();
            Checking();
        }
        public static void sendreceive()
        {
            _receiveTokenSource = new CancellationTokenSource();
            _buffer = new Queue<object>();
            HubMsg hub = new HubMsg(_buffer);
             _getSendingTask = GetDate(_receiveTokenSource.Token);
            _receiveTask = hub.Receive(_receiveTokenSource.Token);
            _sendTask = hub.Sending(_receiveTokenSource.Token);
        }

        public static async Task GetDate(CancellationToken token)
        {
            while (!token.IsCancellationRequested)
            {
                DateTime date = DateTime.Now;
                Data d = new Data();
                d.valueTemp = Bluetooth.temperature;
                d.Lat = Bluetooth.lat;
                d.Lon = Bluetooth.lon; 
                d.valueDateTime = date.ToString();
                d.valueID = DeviceID;
                d.WaterLevel = WaterControl.status;
                lock (_buffer)
                    {
                        _buffer.Enqueue(d);
                    }
                await Task.Delay(3000);
            }
        }

        private void Checking()
        {
            InitGpio._Button1.DebounceTimeout = TimeSpan.FromMilliseconds(100);
            InitGpio._Button1.ValueChanged += buttonPin_ValueChanged;
        }

        private async void buttonPin_ValueChanged(GpioPin sender, GpioPinValueChangedEventArgs e)
        {

            if (i == 0)
            {
                if (onof == false)
                {
                    onof = true;
                    await blink(InitGpio._HubStatePin,4);
                    sendreceive();
                }
                else
                {
                    
                }
                i++;
            }
            else i--;
        }
        public static async Task blink(GpioPin pin,int count)
        {
            int k = 0;
            while (k < count)
            {
                pin.Write(GpioPinValue.High);
                await Task.Delay(500);
                pin.Write(GpioPinValue.Low);
                await Task.Delay(500);
                k++;
            }
        }
    }
}