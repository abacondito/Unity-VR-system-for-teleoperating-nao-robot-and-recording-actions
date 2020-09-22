using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class KeyBoardCrouch : MonoBehaviour
{

    float crouchValue = 0f;

    // Start is called before the first frame update
    void Start()
    {

    }

    // Update is called once per frame
    void FixedUpdate()
    {
        float crouch;

        crouch = Input.GetAxis("Crouch");

        if (crouch != 0)
        {
            crouchValue = crouchValue + -(crouch * 1.2f);
            if (crouchValue > 120f) crouchValue = 120f;

            if (crouchValue < 0f) crouchValue = 0f;

        }

        //UnityEngine.Debug.Log(crouchValue);
    }

    public float GetCrouch()
    {

        //UnityEngine.Debug.Log(crouchValue);

        return crouchValue;
    }

    public void ResetCrouch()
    {
        crouchValue = 0f;
    }
}
