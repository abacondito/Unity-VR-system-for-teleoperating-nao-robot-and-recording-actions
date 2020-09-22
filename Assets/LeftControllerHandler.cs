using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR;

public class LeftControllerHandler : MonoBehaviour
{
    private InputDevice targetDevice;

    // Start is called before the first frame update
    void Start()
    {
        TryInitialize();
    }



    // Update is called once per frame
    void Update()
    {
        if (!targetDevice.isValid)
        {
            TryInitialize();
        }
 

        targetDevice.TryGetFeatureValue(CommonUsages.grip, out float gripValue);

        if(gripValue > 0.1f)
        {
            Debug.Log("Left controller Grip pressed " + gripValue);
        }
            

        targetDevice.TryGetFeatureValue(CommonUsages.primary2DAxis, out Vector2 primary2DAxisValue);

        if (primary2DAxisValue != Vector2.zero)
        {
            Debug.Log("Left controller Stick" + primary2DAxisValue);
        }
            

    }

    void TryInitialize()
    {
        List<InputDevice> devices = new List<InputDevice>();
        InputDeviceCharacteristics controllerCharacteristics = InputDeviceCharacteristics.Left | InputDeviceCharacteristics.Controller;
        InputDevices.GetDevicesWithCharacteristics(controllerCharacteristics, devices);

        if (devices.Count > 0)
        {
            targetDevice = devices[0];
        }
    }

    public Vector2 GetStickValue()
    {
        targetDevice.TryGetFeatureValue(CommonUsages.primary2DAxis, out Vector2 primary2DAxisValue);

        return primary2DAxisValue;
    }

    public float GetGripValue()
    {
        targetDevice.TryGetFeatureValue(CommonUsages.grip, out float gripValue);

        return gripValue;
    }
}
