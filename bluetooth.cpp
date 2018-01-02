				Name of Ble device needs to be set by the user when calling the constructor, and set UUID of service/characteristic
///				Looks for the following service and characteristic
///				2456E1B9-26E2-8F83-E744-F34F01E9D701 --> Service
///				2456E1B9-26E2-8F83-E744-F34F01E9D703 --> Characteristic
///				Some elements on the array might be duplicates
///				
///             !!!!UWP FOR DESKTOP NuGet package needed to work!!!
///				

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using Windows.Devices.Bluetooth;
using Windows.Devices.Bluetooth.Advertisement;
using Windows.Devices.Bluetooth.GenericAttributeProfile;
using Windows.Storage.Streams;




namespace Rainin.Develpment.RAP
{
    public class BLE:RemoteDataLink
    {
        //Used to be $-->'$'
        //Used to be $-->(char)0x0D
        public const char SOF_CHAR = (char)0x12;		// start of frame character
        public const char SOF_CS_CHAR = '%';	// start of frame character (checksum frame)
        public const char FS_CHAR = ',';		// field separator
        public const char CS_CHAR = ';';		// checksum separator
        public const char SD_CHAR = '"';		// string delimiter
        public const char MN_CHAR = ':';		// message number separator
        public const char EOF_CHAR = (char)0x00;  // end of frame character (ASCII CR)
        public const char ASCII_CR = (char)0x0D;
        public const char ASCII_LF = (char)0x0A;

        //State Values
        private const int MAX_BUFFER_SIZE = 1024;
        public uint bytesDiscarded = 0;
        //State Variables
        private int inState;
        private uint inputIndex;
        private byte[] inputBuffer = new byte[MAX_BUFFER_SIZE];
        //BLE Variables
        public string NameOfDevice;
        int i = 0;//used to add elements to _btInfoList;
        int numberofdevices = 0;//used to count number of devices inside _btInfolist
        public static BluetoothLEAdvertisementReceivedEventArgs[] _btInfoList = new BluetoothLEAdvertisementReceivedEventArgs[100];//size of array set to 100
        IReadOnlyList<GattDeviceService> _services;
        GattDeviceService _service;
        GattCharacteristic _characteristic;
        BluetoothLEDevice newdevice;
        // Looks for the following service
        //private Guid service3 = new Guid("2456E1B9-26E2-8F83-E744-F34F01E9D701");
        Guid Service;
        Guid Characteristic;
        int messageCount;
        // statistics
        public uint packetsReceived = 0;
        public uint packetsSent = 0;
        public uint checkSumFailures = 0;
        public uint frameOverruns = 0;
        public uint frameUnderruns = 0;
        public uint bytesReceived = 0;
   
        MessageHandler messageHandler;	// received packet handler
        MonitorOutput monitorOutput;	// user output monitor
        MonitorInput monitorInput;		// user input monitor
        //Timer
        Stopwatch stopWatchB = new Stopwatch();

        int packetCount = 0;

        int baudRate;					// unused data

        bool data;
        //semaphore
        static Semaphore semaphore = new Semaphore(1, 1);

        /// <summary>
        /// Constructor
        /// </summary>

        public BLE(string name, Guid ServiceID, Guid CharacteristicID)
        {
            //Set Buffer
           
            Service = ServiceID;
            Characteristic = CharacteristicID;
            NameOfDevice = name;
            var watcher = new BluetoothLEAdvertisementWatcher();

            watcher.ScanningMode = BluetoothLEScanningMode.Active;

            // Only activate the watcher when we're recieving values >= -80
            watcher.SignalStrengthFilter.InRangeThresholdInDBm = -80;

            // Stop watching if the value drops below -90 (user walked away)
            watcher.SignalStrengthFilter.OutOfRangeThresholdInDBm = -90;

            // Register callback for when we see an advertisements
            watcher.Received += OnAdvertisementReceived;

            // Wait 5 seconds to make sure the device is really out of range
            watcher.SignalStrengthFilter.OutOfRangeTimeout = TimeSpan.FromMilliseconds(5000);
            watcher.SignalStrengthFilter.SamplingInterval = TimeSpan.FromMilliseconds(2000);

            // Starting watching for advertisements
            watcher.Start();

        }
        /// <summary>
        /// Looks for devices (--->Discovery function)
        /// </summary>
        /// <param name="watcher"></param>
        /// <param name="eventArgs"></param>

        private void OnAdvertisementReceived(BluetoothLEAdvertisementWatcher watcher, BluetoothLEAdvertisementReceivedEventArgs eventArgs)
        {
            Debug.WriteLine(String.Format("  BT_ADDR: {0}", eventArgs.BluetoothAddress));
            Debug.WriteLine(String.Format("  FR_NAME: {0}", eventArgs.Advertisement.LocalName));

            //string name = "TEST1";
            
            string namet = eventArgs.Advertisement.LocalName;
            Debug.WriteLine("lengthlocalname: {0} ", namet.Length);
            bool result = string.Equals(namet, NameOfDevice, StringComparison.Ordinal);
            Debug.WriteLine(result.ToString());
            if (result)
            {
                watcher.Stop();
                _btInfoList[i] = eventArgs;
                COUNT();
                ConnectDevice(eventArgs);
            }
            _btInfoList[i] = eventArgs;
            i++;
        }
        /// <summary>
        /// Connects to Desired BLE device
        /// </summary>
        /// <param name="eventArgs"></param>
        async void ConnectDevice(BluetoothLEAdvertisementReceivedEventArgs eventArgs)
        {
            BluetoothLEDevice device = await BluetoothLEDevice.FromBluetoothAddressAsync(eventArgs.BluetoothAddress);
            var id = device.DeviceId;

            Debug.WriteLine("Device Id: ", id.ToString());
            newdevice = await BluetoothLEDevice.FromIdAsync(device.DeviceId);
            device.Dispose();
            device = null;
            IReadOnlyCollection<GattDeviceService> rx = newdevice.GattServices;
            Debug.WriteLine("DEvice name  ", newdevice.Name);
            Debug.WriteLine("Number of Services: ", newdevice.GattServices.Count.ToString());
            _services = newdevice.GattServices;

            //Gets all the services from device
            foreach (var ser in _services)
            {
                Debug.WriteLine("service uuid {0}", ser.Uuid);
                //Connects to desired GUUID
                if (ser.Uuid.Equals(Service))//--> change to Service
                {
                    Debug.WriteLine("CONNECTED TO SERVICE: {0}", ser.Uuid);
                    _service = ser;
                    break;
                }
            }
            //Looks and conncets to desired GUUID within Service
            foreach (var c in _service.GetCharacteristics(Characteristic))//new Guid("2456E1B9-26E2-8F83-E744-F34F01E9D703")))//-->Change to Characteristic
            {
                _characteristic = c;
                Debug.WriteLine("Characteristic found {0}  ", c.Uuid);
            }
            Properties();
        }
        /// <summary>
        /// Checks Characteristic Properties
        /// </summary>
        void Properties()
        {
            GattCharacteristicProperties properties = _characteristic.CharacteristicProperties;
            if (properties.HasFlag(GattCharacteristicProperties.Write))
            {
                Debug.WriteLine("Supports writing");
                // This characteristic supports writing to it.

            }
            if (properties.HasFlag(GattCharacteristicProperties.Read))
            {
                Debug.WriteLine("Supports Reading");
                // This characteristic supports reading from it.
            }
            if (properties.HasFlag(GattCharacteristicProperties.Notify))
            {
                Debug.WriteLine("Supports Subscribing notifications");
                // This characteristic supports subscribing to notifications.
            }
        }
        
        /// <summary>
        /// Checks for number of devices inside _btInfoList
        /// </summary>
        /// <returns></returns>
        public int COUNT()
        {

            for (int i = 0; i < 30; i++)
            {
                if (_btInfoList[i] == null)
                {
                    return numberofdevices;
                    //break;
                }
                else
                {
                    Debug.WriteLine("Number of devices {0} ", numberofdevices);
                    numberofdevices = numberofdevices + 1;
                }
            }
                return numberofdevices; 
        }

        /// <summary>
        /// Set specific service and characteristic to look for
        /// </summary>
        /// <param name="addr"></param>
        public void SetUUid(string SerUuid, string CharUuid)
        {
             Service = new Guid(SerUuid);
             Characteristic = new Guid(CharUuid);
        }


        /// <summary>
        /// Returns number of devices found --Some devices might have been added to the list twice
        /// </summary>
        public int DeviceCount
        {
            get
            {
                if (_btInfoList != null) return numberofdevices;
                else return 0;
            }
        }

        /// <summary>
        /// Checks if any devices were found 
        /// </summary>
        /// <returns></returns>

        public bool DevicesAvailable()
        {
            if (_btInfoList == null)
            {
                return false;
            }
            else
            {
                return true;
            }
        }
        /// <summary>
        /// Returns Local Name of Devices found
        /// </summary>
        /// <returns></returns>

        public string[] DeviceListByName()
        {
            if (_btInfoList == null) return null;
            int count = numberofdevices;
            string[] list = new string[count];


            for (int i = 0; i < count; i++)
            {
                list[i] = _btInfoList[i].Advertisement.LocalName;
            }

            return list;
        }

        /// <summary>
        /// Returns BLE Adresses of devices found
        /// </summary>
        /// <returns></returns>

        public string[] DeviceListByAddress()
        {
            if (_btInfoList == null) return null;
            int count = numberofdevices;
            string[] list = new string[count];

            for (int i = 0; i < count; i++)
            {

                list[i] = _btInfoList[i].BluetoothAddress.ToString();

            }

            return list;
        }
        /// <summary>
        /// Receives Notifications from BLE Device
        /// </summary>
        /// 

        async public void Notification()
        {
            //semaphore.WaitOne();
            GattCommunicationStatus status = await _characteristic.WriteClientCharacteristicConfigurationDescriptorAsync(GattClientCharacteristicConfigurationDescriptorValue.Notify);
            if (status == GattCommunicationStatus.Success)
            {
                
                Debug.WriteLine("Notification Received");
                _characteristic.ValueChanged += Characteristic_Valuechanged;
                
            }
            else{
                 Debug.WriteLine("Notification NOT Received");

            }
            stopWatchB.Stop();
            Debug.WriteLine("TIME SINCE start of receiving thread {0}", stopWatchB.Elapsed);
        //    semaphore.Release();

        }
        //DO something with notifications
        void Characteristic_Valuechanged(GattCharacteristic sender, GattValueChangedEventArgs args)
        {
            //Indicates repor
            var reader = DataReader.FromBuffer(args.CharacteristicValue);
            byte[] input = new byte[reader.UnconsumedBufferLength];   
            reader.ReadBytes(input);
            Debug.WriteLine("Received: ", BitConverter.ToString(input));
        }


        /// <summary>
        /// Recieves data from ble device
        /// </summary>
        /// 
       async public void DataReceived()
        {  
            

            GattCharacteristicProperties properties = _characteristic.CharacteristicProperties;
            if (properties.HasFlag(GattCharacteristicProperties.Read))
            {
                semaphore.WaitOne();
                // This characteristic supports reading from it.
               // Debug.WriteLine("Supports Reading");
                
                var result = await _characteristic.ReadValueAsync();
                Debug.WriteLine("BufferSize: ", result.Value);
                var reader = DataReader.FromBuffer(result.Value);
                byte[] input = new byte[reader.UnconsumedBufferLength];
                //inputBuffer = new byte[reader.UnconsumedBufferLength];
                reader.ReadBytes(input);
                int size = input.Length;
                Debug.WriteLine("Size:", size.ToString());
                Debug.WriteLine("Received: ", BitConverter.ToString(input));
                for (int count = 0; count < size; count++)
                {
                  ProcessByte(input[count]);
                   ++packetCount;
                }
                stopWatchB.Stop();
                Debug.WriteLine("TIME SINCE start of receiving thread {0}", stopWatchB.Elapsed);
                semaphore.Release();
            }
            else
            {
                Debug.WriteLine("Does not support Reading");
            }
           
           
        }
        /// <summary>
        /// Porcess one byte of input data
        /// </summary>
        public const int STATE_INIT = 0;
        public const int STATE_WAIT_SOF = 1;
        public const int STATE_WAIT_EOF = 2;
        public const int STATE_WAIT_EOF_CS = 3;


        private void ProcessByte(byte data)
        {
            bytesReceived++;
            Debug.WriteLine("Proccess Following Byte:{0} ", Convert.ToInt16(data));//.ToString());//,Encoding.Default.GetString(data));

            if (monitorInput != null)
            {
                if (data == (byte)ASCII_CR)
                {
                    monitorInput((byte)'<');
                    monitorInput((byte)'C');
                    monitorInput((byte)'R');
                    monitorInput((byte)'>');
                    monitorInput(data);
                }
                else if (data == (byte)ASCII_LF)
                {
                    monitorInput((byte)'<');
                    monitorInput((byte)'L');
                    monitorInput((byte)'F');
                    monitorInput((byte)'>');
                }
                else if (data == (byte)0)
                {
                    monitorInput((byte)'<');
                    monitorInput((byte)'0');
                    monitorInput((byte)'>');
                }
                else monitorInput(data);
            }

 
            switch (inState)
            {
                case STATE_INIT:
                    // start a new message
                    inputIndex = 0;
                    inState = STATE_WAIT_SOF;
                    goto case STATE_WAIT_SOF;

                case STATE_WAIT_SOF:
                    Debug.WriteLine("STATE_WAIT_SOF:{0} ", data);
                    // wait for a start of frame char
                    if (data == SOF_CHAR)
                    {
                        inState = STATE_WAIT_EOF;
                    }
                    else if (data == SOF_CS_CHAR)
                    {
                        // inputBuffer[inputIndex++] = data;
                        inState = STATE_WAIT_EOF_CS;
                    }
                    else
                    {
                        bytesDiscarded++;
                    }
                    break;

                case STATE_WAIT_EOF:
                    // record all...
                    if (inputIndex < MAX_BUFFER_SIZE)
                    {
                        Debug.WriteLine("STATE_WAIT_EOF:{0} ", data);//BitConverter.ToString(data));
                        inputBuffer[inputIndex++] = data;
                    }
                    else // overrun
                    {
                        // discard data and start over
                        bytesDiscarded += inputIndex;
                        inState = STATE_INIT;
                    }
                    if (data == EOF_CHAR)
                    {
                        // terminate the message
                        inputBuffer[inputIndex - 1] = 0;


                      //  if (messageHandler != null)
                     //   {
                    //        messageHandler(inputBuffer, (int)inputIndex);
                     //   }
                        inState = STATE_INIT;
                    }
                    break;

                case STATE_WAIT_EOF_CS:
                    // record all...
                    if (inputIndex < MAX_BUFFER_SIZE)
                    {
                        Debug.WriteLine("STATE_EOF_CS:{0} ", data);//BitConverter.ToString(data));
                        inputBuffer[inputIndex++] = data;
                    }
                    else // overrun
                    {
                        // discard data and start over
                        bytesDiscarded += inputIndex;
                        inState = STATE_INIT;
                    }
                    if (data == EOF_CHAR)
                    {
                        //

                        //
                        if (TestChecksum(inputBuffer, inputIndex))
                        {
                            //
                            string s = System.Text.Encoding.ASCII.GetString(inputBuffer);
                            inputBuffer[inputIndex - 6] = 0;


                          //  if (messageHandler != null)
                         //   {
                         //       messageHandler(inputBuffer, (int)inputIndex);
                         //       ++messageCount;
                          //  }
                        }
                        else
                        {
                            // break here if communication error
                            checkSumFailures++;
                            Debug.WriteLine("CRC Error detected");
                        }
                        // start a new message
                        inState = STATE_INIT;
                    }
                    break;
                default:
                    throw new Exception("Invalid state");
            }
        }

        public void BytesDiscarded()
        {

            Console.WriteLine("Discarded Bytes {0}", bytesDiscarded);
        }

        /// <summary>
        /// Send one byte of data
        /// </summary>
        /// <param name="characteristic"></param>
        async public void Send(byte datatosend)
        {
            semaphore.WaitOne();
            stopWatchB.Reset();
            stopWatchB.Start();
            
            DataWriter writer = new DataWriter();
            byte[] data = new byte[] { datatosend };
            writer.WriteBytes(data);
            Debug.WriteLine("Write SUCCESS: ", BitConverter.ToString(data));
            GattCommunicationStatus result = 0;
            var buf = writer.DetachBuffer();
            result = await _characteristic.WriteValueAsync(buf);
            semaphore.Release();
            if (_characteristic.CharacteristicProperties.HasFlag(GattCharacteristicProperties.Notify))
            {
                Debug.WriteLine("Sending to notify function");
                Notification();
                // This characteristic supports subscribing to notifications.
            }
        }
        /// <summary>
        /// Send a byte of data
        /// </summary>
        /// <param name="data"></param>
        public void SendByte(byte data)
        {
            Send(data);
            if (monitorOutput != null)
            {
                if (data == (byte)ASCII_CR)
                {
                    monitorOutput((byte)'<');
                    monitorOutput((byte)'C');
                    monitorOutput((byte)'R');
                    monitorOutput((byte)'>');
                }
                else if (data == (byte)ASCII_LF)
                {
                    monitorOutput((byte)'<');
                    monitorOutput((byte)'L');
                    monitorOutput((byte)'F');
                    monitorOutput((byte)'>');
                }
                monitorOutput(data);
            }
        }


        public void InformationReceived(){

            
            foreach (var element in inputBuffer)
            {
                if (element != 0)
                {
                    Console.WriteLine("data inside inputBuffer {0} ", element);
                }
             }
        }
   
        /// <summary>
        /// Sets/gets Device name
        /// </summary>
        public string PortName
        {
            set
            {
               ///I do not know how to set this  devicePath = _ipServerAddress.ToString() + ":" + _ipServerPort;
            }
            get
            {
                return NameOfDevice;
            }
        }
        /// <summary>
        /// Baud rate - OBSOLETE
        /// </summary>
        public int BaudRate
        {
            set
            {
                baudRate = value;
            }
            get
            {
                return baudRate;
            }
        }
        /// <summary>
        /// Checks connectivity by checking if the ble object is null
        /// </summary>
        /// <returns></returns>
        public bool Open()
        {

            if (newdevice == null)
            {
                return false;
            }
            else{
                return true;
            }
        }
        /// <summary>
        /// Close the data link device.
        /// </summary>
        public void Close()
        {
            if (newdevice != null)
                {
                    newdevice.Dispose();
                    newdevice = null;
                }
        }
        /// <summary>
        /// Reset statistics.
        /// </summary>
        public void ResetStatistics()
        {
            packetsReceived = 0;
            packetsSent = 0;
            checkSumFailures = 0;
            frameOverruns = 0;
            frameUnderruns = 0;
        }

        /// <summary>
        /// Adds a message handler
        /// </summary>
        /// <param name="handler"></param>
        public void AddMessageHandler(MessageHandler handler)
        {
            messageHandler += handler;
        }

        public void AddInputMonitor(MonitorInput inputMon)
        {
            monitorInput += inputMon;
        }

        public void AddOutputMonitor(MonitorOutput outputMon)
        {
            monitorOutput += outputMon;
        }

        /// <summary>
        /// Establishes if byte is a "valid" ASCII character
        /// </summary>
        /// <param name="value"></param>
        /// <returns></returns>
        private bool IsASCIIHexDigit(byte value)
        {
            return (((value >= '0') && (value <= '9'))
            || ((value >= 'A') && (value <= 'F'))
            || ((value >= 'a') && (value <= 'f')));
        }

        /// <summary>
        /// Format and send a string of bytes with the checksum if requested
        /// </summary>
        /// <param name="toSend">String to be sent.</param>
        public bool SendData(byte[] toSend, bool addChecksum)
        {
            bool sentOK = false;
            try
            {
                if (addChecksum)
                {
                    // compute the checksum
                    ushort Crc = Crc16(toSend, 0, (uint)toSend.Length);
                    // send the start char
                    SendByte((byte)SOF_CS_CHAR);
                    // send the frame
                    SendBytes(toSend);
                    // the checksum delimiter
                    SendByte((byte)CS_CHAR);
                    // format the checksum
                    byte[] hexBytes = Encoding.ASCII.GetBytes(String.Format("{0:x4}", Crc));
                    SendBytes(hexBytes);
                    // send the end char
                    SendByte((byte)EOF_CHAR);
                }
                else
                {
                    // send the start char
                    SendByte((byte)SOF_CHAR);
                    // send the frame
                    SendBytes(toSend);
                    // send the end char
                    SendByte((byte)EOF_CHAR);
                }   
                sentOK = true;
            }
            catch (Exception)
            {
                // todo: log error here
            }
            return sentOK;
        }

        /// <summary>
        /// Test the checksum
        /// </summary>
        /// <param name="inputBuffer"></param>
        /// <param name="length"></param>
        /// <returns></returns>
        private bool TestChecksum(byte[] inputBuffer, uint length)
        {
            bool status = false;
            // message must have 5 checksum digits plus 
            if ((length > 6)
                && (inputBuffer[length - 6] == ';')
                && IsASCIIHexDigit(inputBuffer[length - 5])
                && IsASCIIHexDigit(inputBuffer[length - 4])
                && IsASCIIHexDigit(inputBuffer[length - 3])
                && IsASCIIHexDigit(inputBuffer[length - 2]))
            {
                // form a string
                StringBuilder sb = new StringBuilder();
                sb = sb.Append((char)inputBuffer[length - 5]);
                sb = sb.Append((char)inputBuffer[length - 4]);
                sb = sb.Append((char)inputBuffer[length - 3]);
                sb = sb.Append((char)inputBuffer[length - 2]);
                string csString = sb.ToString();
                //Debug.WriteLine("crc=" + csString);
                // convert base 16
                try
                {
                    ushort msgcs = Convert.ToUInt16(csString, 16);
                    ushort cmpcs = Crc16(inputBuffer, 0, length - 6);
                    if (msgcs == cmpcs)
                    {
                        status = true;
                    }
                }
                catch (Exception)
                {
                    // if bad convert, ignore message
                }
            }

            return status;
        }

        private const ushort CRC_INIT = 0xffff;
        private const ushort CRC_PASS = 0xf0b8;

        // CRC-16 lookup tables
        private static ushort[] CrcTableA = new ushort[] {
            0x0000, 0x1189, 0x2312, 0x329b, 
            0x4624, 0x57ad, 0x6536, 0x74bf,
            0x8c48, 0x9dc1, 0xaf5a, 0xbed3,
            0xca6c, 0xdbe5, 0xe97e, 0xf8f7
            };
        private static ushort[] CrcTableB = new ushort[] {
            0x0000, 0x1081, 0x2102, 0x3183,
            0x4204, 0x5285, 0x6306, 0x7387,
            0x8408, 0x9489, 0xa50a, 0xb58b,
            0xc60c, 0xd68d, 0xe70e, 0xf78f
            };

        /// <summary>
        /// Calculates CRC-16 on the buffer provided.
        /// </summary>
        /// <param name="buffer"></param>
        /// <param name="length"></param>
        /// <returns></returns>
        public ushort Crc16(byte[] buffer, uint start, uint length)
        {
            byte Index;
            ushort Crc = CRC_INIT;
            for (int i = 0; i < length; i++)
            {
                Index = (byte)(Crc ^ buffer[i + start]);
                Crc = (ushort)((Crc >> 8) ^ CrcTableA[Index & 0x0f] ^ CrcTableB[Index >> 4]);
            }
            return Crc;
        }

        /// <summary>
        /// Send a byte array of data
        /// </summary>
        /// <param name="data"></param>
        public void SendBytes(byte[] data)
        {
            for (int i = 0; i < data.Length; i++)
            {
                SendByte(data[i]);
            }
        }
    }
}
