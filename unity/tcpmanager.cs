using System;
using System.Collections.Generic;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;
using Newtonsoft.Json;

public class TCPManager : MonoBehaviour
{
    public static TCPManager Instance { get; private set; } // Singleton Instance
    public Dictionary<string, string> SensorData { get; private set; }  // Stores sensor readings

    private TcpClient client;
    private NetworkStream stream;
    private Thread receiveThread;
    private bool isRunning = false;

    void Awake()
    {
        if (Instance == null)
        {
            Instance = this;
            DontDestroyOnLoad(this.gameObject);
            SensorData = new Dictionary<string, string>(); // Stores sensor data
        }
        else
        {
            Destroy(this.gameObject);
        }
    }

    void Start()
    {
        ConnectToESP();
    }

    private void ConnectToESP()
    {
        try
        {
            client = new TcpClient("192.168.4.1", 80); // Replace with NodeMCU IP and port
            stream = client.GetStream();
            isRunning = true;

            receiveThread = new Thread(ReceiveMessages);
            receiveThread.IsBackground = true;
            receiveThread.Start();

            Debug.Log("Connected to NodeMCU.");
        }
        catch (Exception e)
        {
            Debug.LogError("Failed to connect to NodeMCU: " + e.Message);
        }
    }


   private void ReceiveMessages()
{
    byte[] buffer = new byte[1024];

    while (isRunning)
    {
        try
        {
            if (stream != null && stream.DataAvailable)
            {
                int bytesRead = stream.Read(buffer, 0, buffer.Length);
                string receivedData = Encoding.UTF8.GetString(buffer, 0, bytesRead);

                // Process only the first JSON object
                string firstJson = ExtractFirstJson(receivedData);

                if (!string.IsNullOrEmpty(firstJson))
                {
                    try
                    {
                        ProcessMessage(firstJson);
                    }
                    catch (Exception e)
                    {
                        Debug.LogError("Failed to process JSON: " + e.Message);
                    }
                }
            }
        }
        catch (Exception e)
        {
            Debug.LogError("Receive error: " + e.Message);
        }

        Thread.Sleep(10);
    }
}

	private string ExtractFirstJson(string data)
{
    int startIndex = data.IndexOf('{'); // Find the first '{'
    int endIndex = data.IndexOf('}');   // Find the first '}'

    if (startIndex != -1 && endIndex != -1 && endIndex > startIndex)
    {
        return data.Substring(startIndex, endIndex - startIndex + 1); // Extract the JSON substring
    }

    return null; // No valid JSON object found
}


    private void ProcessMessage(string message)
{
    message = message.Trim();

    if (string.IsNullOrWhiteSpace(message)) return;

    try
    {
        var parsedData = JsonConvert.DeserializeObject<Dictionary<string, string>>(message);

        if (parsedData != null)
        {
            foreach (var item in parsedData)
            {
                SensorData[item.Key] = item.Value;
            }

            Debug.Log("Processed JSON: " + message);
        }
    }
    catch (JsonException e)
    {
        Debug.LogError("JSON parsing error: " + e.Message);
    }
}


     void OnDestroy()
    {
        isRunning = false;
        receiveThread?.Join();
        stream?.Close();
        client?.Close();

        Debug.Log("TCP Connection closed.");
    }
}