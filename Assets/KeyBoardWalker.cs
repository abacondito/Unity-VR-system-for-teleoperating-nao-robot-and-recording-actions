using System.Collections;
using System.Collections.Generic;
using System.Security.Cryptography.X509Certificates;
using UnityEngine;

public class KeyBoardWalker : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {

    }

    // Update is called once per frame
    void Update()
    {

    }

    public Vector2 GetMovement()
    {

        Vector2 vector;

        vector.x = Input.GetAxis("Horizontal");
        vector.y = Input.GetAxis("Vertical");

        vector = vector.normalized;


        return vector;
    }
}
