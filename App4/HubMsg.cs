using Microsoft.Azure.Devices.Client;
using Newtonsoft.Json;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using Windows.Devices.Gpio;

namespace App4
{
    class HubMsg
    {
        public static CancellationTokenSource _receiveTokenSource2;
        public static Task _AutoWater;
        Queue<object> _buffer;
        public HubMsg(Queue<object> buffer)
        {
            _buffer = buffer;
        }
        public static DeviceClient deviceClient;
        public static string FirstDevice = "HostName=AVN-group.azure-devices.net;DeviceId=RaspberyData;SharedAccessKey=en9r0KNg4+zG15msYX/aFjtut4n0R+2fmi3iVU9GC+E=";

        public async Task Sending(CancellationToken token)
        {
           
                deviceClient = DeviceClient.CreateFromConnectionString(FirstDevice, TransportType.Http1);

                while (!token.IsCancellationRequested)//!token.IsCancellationRequested
                {
                    try
                    {
                        List<object> dBuffer = new List<object>();
                        lock (_buffer)
                        {
                            while (_buffer.Count > 0)
                            {
                                var d = _buffer.Dequeue();
                                dBuffer.Add(d);
                            }
                        }
                        foreach (var d in dBuffer)
                        {
                            var msg = new Message(Encoding.UTF8.GetBytes(JsonConvert.SerializeObject(d)));
                            await deviceClient.SendEventAsync(msg);
                            InitGpio._HubStatePin.Write(GpioPinValue.High);
                        }
                    }
                    catch (Exception)
                    {
                    }
                    await Task.Delay(3000);
                }
            
        }

        public async Task Receive(CancellationToken token)
        {
            deviceClient = DeviceClient.CreateFromConnectionString(FirstDevice, TransportType.Http1);
            while (!token.IsCancellationRequested)//!token.IsCancellationRequested
            {
                try
                {
                    Message receivedMessage = await deviceClient.ReceiveAsync();
                    if (receivedMessage != null)
                    {
                        string Command = Encoding.ASCII.GetString(receivedMessage.GetBytes());
                        switch (Command)
                        {
                            case "1":
                                
                                InitGpio._relePin.Write(GpioPinValue.High);
                                break;
                            case "-1":
                                InitGpio._relePin.Write(GpioPinValue.Low);
                                break;
                            case "4":
                                _receiveTokenSource2 = new CancellationTokenSource();
                                _AutoWater = WaterControl.AutoWater(_receiveTokenSource2.Token);
                                break;
                            case "-4":

                                if (_receiveTokenSource2 != null)
                                {
                                    _receiveTokenSource2.Cancel();
                                }
                                break;
                            default:
                                break;
                        }
                        await deviceClient.CompleteAsync(receivedMessage);
                    }
                }
                catch(Exception exc)
                {
                    Debug.Write(exc.Message);
                }
                await Task.Delay(500);
            }

        }
    }
}
