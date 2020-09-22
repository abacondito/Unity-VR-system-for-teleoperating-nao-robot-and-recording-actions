using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR;

public class RightControllerHandler : MonoBehaviour
{
    private InputDevice targetDevice;

    float crouchValue = 0f;

    // Start is called before the first frame update
    void Start()
    {
        //UnityEngine.XR.InputTracking.disablePositionalTracking = true;
        TryInitialize();
    }



    // Update is called once per frame
    void FixedUpdate()
    {
        if (!targetDevice.isValid)
        {
            TryInitialize();
        }


        targetDevice.TryGetFeatureValue(CommonUsages.grip, out float gripValue);

        if (gripValue > 0.1f)
        {
            Debug.Log("Right controller Grip pressed " + gripValue);
        }


        targetDevice.TryGetFeatureValue(CommonUsages.primary2DAxis, out Vector2 primary2DAxisValue);

        if (primary2DAxisValue != Vector2.zero)
        {
            Debug.Log("Right controller Stick" + primary2DAxisValue);
        }

        float crouch = primary2DAxisValue.y;



        if (crouch != 0)
        {
            crouchValue = crouchValue + -(crouch * 1.2f);
            if (crouchValue > 120f) crouchValue = 120f;

            if (crouchValue < 0f) crouchValue = 0f;

        }
    }

    void TryInitialize()
    {
        List<InputDevice> devices = new List<InputDevice>();
        InputDeviceCharacteristics controllerCharacteristics = InputDeviceCharacteristics.Right | InputDeviceCharacteristics.Controller;
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

    public float getCrouch()
    {
        return crouchValue;
    }

    public void resetCrouch()
    {
        crouchValue = 0f;
    }
}
