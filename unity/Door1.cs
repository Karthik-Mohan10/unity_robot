using UnityEngine;

public class Door1 : Door
{

    private bool isDoorOpen = false; // Tracks the current state of the door
    private string previousIRValue = ""; // Tracks the previous IR sensor value
    
    protected override void IRsensorcheck()
    {

        //base.Update(); // Use the main Door class logic for range and toggling

        if (TCPManager.Instance != null)
        {
            string irValue;

            // Access the shared SensorData safely
            lock (TCPManager.Instance.SensorData)
            {
                TCPManager.Instance.SensorData.TryGetValue("IR", out irValue);
            }

            if (!string.IsNullOrEmpty(irValue))
            {
                // Door opens when IR sensor reads "CLOSE" for the first time
                if (irValue == "CLOSE" && previousIRValue != "CLOSE" && !isDoorOpen)
                {
                    ToggleDoor();  // Call the ToggleDoor() method to open
                    isDoorOpen = true; // Update the door state
                }
                // Door closes when IR sensor reads "CLOSE" after "OPEN"
                else if (irValue == "CLOSE" && previousIRValue == "OPEN" && isDoorOpen)
                {
                    ToggleDoor();  // Call the ToggleDoor() method to close
                    isDoorOpen = false; // Update the door state
                }

                // Update the previous IR value for comparison
                previousIRValue = irValue;
            }
        }
    }
}
