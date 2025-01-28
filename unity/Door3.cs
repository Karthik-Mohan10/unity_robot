using UnityEngine;

public class Door3 : Door
{

    private bool isDoorOpen = false; // Tracks the current state of the door
    private string previousIRValue = ""; // Tracks the previous IR sensor value
    
    protected override string IRsensorcheck()
    {
        string t_status = "no";     // Variable to return the IR sensor status

        if (TCPManager.Instance != null)
        {
            string irValue;        // Variable to store IR sensor value from Arduino

            // Access the shared SensorData safely
            lock (TCPManager.Instance.SensorData)
            {
                TCPManager.Instance.SensorData.TryGetValue("IR", out irValue);
            }

            if (!string.IsNullOrEmpty(irValue))
            {
                // Door opens when IR sensor outputs "CLOSE" for the first time and for further obstacle detections
                if (irValue == "CLOSE" && previousIRValue != "CLOSE" && !isDoorOpen)
                {
                    
                    isDoorOpen = true;  // Update the door status
                    t_status = "yes";   // Set the return value

                }
                // Door closes when IR sensor outputs "CLOSE" after "OPEN"
                else if (irValue == "CLOSE" && previousIRValue == "OPEN" && isDoorOpen)
                {
                    
                    isDoorOpen = false;  // Update the door status
                    t_status = "yes";  // Set the return value

                }
                // Update the previous IR value for comparison
                previousIRValue = irValue;   
            }
        }
        return t_status;
    }
}
