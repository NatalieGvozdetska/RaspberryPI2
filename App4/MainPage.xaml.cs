﻿using System;
using System.Threading;
using System.Threading.Tasks;
using Windows.Devices.Gpio;
using System.Collections.Generic;

namespace PuppyCareApp
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


        public MainPage()
        {
            //Initiailizing pins
            InitGpio.Init();
            InitializeComponent();
        }
        public static void sendreceive()
        {
            _receiveTokenSource = new CancellationTokenSource();
            _buffer = new Queue<object>();
            IoTHub hub = new IoTHub(_buffer);
             _getSendingTask = FormJSONobj2send(_receiveTokenSource.Token);
            _receiveTask = hub.Receive(_receiveTokenSource.Token);
            _sendTask = hub.Send(_receiveTokenSource.Token);
        }

        public static async Task FormJSONobj2send(CancellationToken token)
        {
            while (!token.IsCancellationRequested)
            {
                DateTime date = DateTime.Now;
                JSONobj d = new JSONobj();
                d.temperature = Bluetooth.temperature;
                d.latitude = Bluetooth.lat;
                d.longitude = Bluetooth.lon; 
                d.datetime = date.ToString();
                d.deviceID = DeviceID;
                lock (_buffer)
                    {
                        _buffer.Enqueue(d);
                    }
                await Task.Delay(3000);
            }
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