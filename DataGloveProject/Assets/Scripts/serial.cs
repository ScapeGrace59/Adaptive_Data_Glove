using System;
using System.Collections;
using System.Collections.Concurrent;
using System.Collections.Generic;
using UnityEngine;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.IO.Ports;
using System.Threading.Tasks;


public class serial : MonoBehaviour
{

  private Thread clientReceiveThread;
  private ConcurrentQueue<float[]> _fingerQueue;
  private ConcurrentQueue<float[]> _rotationQueue;
  private ConcurrentQueue<float[]> _translationQueue;
  SerialPort stream;
  handControl control;
  public string portName; //must set port name within unity editor to run

  float[] lastFingerValue;
  float[] lastHandRotationValue;
  float[] lastHandTranslationValue;

  // Start is called before the first frame update
  void Start()
  {
    _fingerQueue = new ConcurrentQueue<float[]>();
    _rotationQueue = new ConcurrentQueue<float[]>();
    _translationQueue = new ConcurrentQueue<float[]>();

    lastFingerValue  = new float[]{0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    lastHandRotationValue  = new float[]{0.0f, 0.0f, 0.0f, 0.0f};
    lastHandTranslationValue  = new float[]{0.0f, 0.0f, 0.0f};

    control = GameObject.Find("ControlHandler").GetComponent<handControl>();

    startCommThread();
  }

	// On ending the Unity game, ensure the serial port is closed properly and end the thread (otherwise you may need to reboot computer)
  void OnApplicationQuit(){
    control.SaveData();
    Debug.Log("ENDING");
    clientReceiveThread.Abort("TEST");
    stream.Close();
  }

	//get last dequeued reading for specified finger 
  public float getFinger(int finger){
    return lastFingerValue[finger];
  }

  public float[] getWrist(){
    return lastHandRotationValue;
  }

  public float[] getHand(){
    return lastHandTranslationValue;
  }
  
	//
  void startCommThread(){
    clientReceiveThread = new Thread(new ThreadStart(ListenForData));
    clientReceiveThread.IsBackground = true;
    Debug.Log("starting handshake");
    clientReceiveThread.Start();
  }

  // Update is called once per frame
  void Update()
  {
    float[] outArray;

    if (_fingerQueue.TryDequeue(out outArray))
    {
      lastFingerValue = outArray;
    }

    if (_rotationQueue.TryDequeue(out outArray))
    {
      lastHandRotationValue = outArray;
    }

    if (_translationQueue.TryDequeue(out outArray))
    {
      lastHandTranslationValue = outArray;
    }
  }

  private void ListenForData()
  {
    stream = new SerialPort(portName, 115200);
    stream.ReadTimeout = 50;
    stream.WriteTimeout = 50;
	
	  //Start serial stream with arduino
    stream.Open();
    stream.Write("g\n\r");

	  //Send a character to arduino to initiate streaming of sensor readings.
    stream.Write("g\n\r");

    while (true)
    {
      if(!stream.IsOpen){
        return;
      }
      string serverMessage = stream.ReadLine();
      string[] messageFilter = serverMessage.Split(';');
      string[] fingerValues = messageFilter[0].Split(',');
      string[] handRotValues = messageFilter[1].Split(',');
      string[] handTransValues = messageFilter[2].Split(',');

      lastFingerValue[0] = float.Parse(fingerValues[0], System.Globalization.CultureInfo.InvariantCulture);
      lastFingerValue[1] = float.Parse(fingerValues[1], System.Globalization.CultureInfo.InvariantCulture);
      lastFingerValue[2] = float.Parse(fingerValues[2], System.Globalization.CultureInfo.InvariantCulture);
      lastFingerValue[3] = float.Parse(fingerValues[3], System.Globalization.CultureInfo.InvariantCulture);
      lastFingerValue[4] = float.Parse(fingerValues[4], System.Globalization.CultureInfo.InvariantCulture);
      lastHandRotationValue[0] = float.Parse(handRotValues[0], System.Globalization.CultureInfo.InvariantCulture);
      lastHandRotationValue[1] = float.Parse(handRotValues[1], System.Globalization.CultureInfo.InvariantCulture);
      lastHandRotationValue[2] = float.Parse(handRotValues[2], System.Globalization.CultureInfo.InvariantCulture);
      lastHandRotationValue[3] = float.Parse(handRotValues[2], System.Globalization.CultureInfo.InvariantCulture);
      lastHandTranslationValue[0] = float.Parse(handTransValues[0], System.Globalization.CultureInfo.InvariantCulture);
      lastHandTranslationValue[1] = float.Parse(handTransValues[1], System.Globalization.CultureInfo.InvariantCulture);
      lastHandTranslationValue[2] = float.Parse(handTransValues[2], System.Globalization.CultureInfo.InvariantCulture);

    }
  }

  private void EnqueueData(string[] values, ConcurrentQueue<float[]> queue, int expectedLength)
    {
        if (values.Length == expectedLength)
        {
            float[] farray = new float[expectedLength];
            for (int i = 0; i < expectedLength; i++)
            {
                float.TryParse(values[i], out farray[i]);
            }

            // Ensure the queue does not grow too large
            if (queue.Count > 1)
            {
                float[] temp;
                queue.TryDequeue(out temp);
            }
            queue.Enqueue(farray);
        }
    }
}