using System.Collections;
using System.Diagnostics;
using System.Collections.Generic;
using System.Net.Sockets;
using System.Text;
using UnityEngine;
using System.Globalization;

public class SocketGatherer : MonoBehaviour
{
    public DitzelGames.FastIK.IKcomplete wristRight;
    public DitzelGames.FastIK.IKcomplete wristLeft;
    public RightControllerHandler rightController;
    public LeftControllerHandler leftController;
    public TestJoint head;

    public KeyBoardWalker keyboardWalk;
    public KeyBoardCrouch keyboardCrouch;

    public ZoraMenuControl canvas;

    TcpClient client;
    StringBuilder myStringBuilderRight;
    StringBuilder myStringBuilderLeft;
    NetworkStream nwStream;
    byte[] bytesToSend;
    public int Port = 9050;

    public string delimiter = ":";

    // Start is called before the first frame update
    void Start()
    {

        client = new TcpClient("127.0.0.1", Port);

        nwStream = client.GetStream();

    }

    void FixedUpdate()
    {

        //initializing the stringbuilder in which x,y,z and axis rotation will be appended
        myStringBuilderRight = new StringBuilder();
        myStringBuilderLeft = new StringBuilder();

        Vector3[] rotationRight = wristRight.getRobotJoints();
        Vector3[] rotationLeft = wristLeft.getRobotJoints();
        Vector3 rotationHead = head.getJoint();
        Vector2 walkDirection = leftController.GetStickValue().normalized;
        //Vector2 walkDirection = keyboardWalk.GetMovement();



        if (walkDirection.x != 0f || walkDirection.y != 0f)
        {
            rightController.resetCrouch();
            //keyboardCrouch.ResetCrouch();
        }

        float crouchValue = rightController.getCrouch();

        UnityEngine.Debug.Log("crouchValue" +  crouchValue);

        //float crouchValue = keyboardCrouch.GetCrouch();

        float rightGrip = rightController.GetGripValue();
        float leftGrip = leftController.GetGripValue();

 

        for (int i = 0; i < rotationRight.Length; i++)
        {

                //appending position and axis rotation with a delimiter to parse them after sent in Bytes
                myStringBuilderLeft.Append(rotationLeft[i].x.ToString(CultureInfo.InvariantCulture) + delimiter);
                myStringBuilderLeft.Append(rotationLeft[i].y.ToString(CultureInfo.InvariantCulture) + delimiter);
                myStringBuilderLeft.Append(rotationLeft[i].z.ToString(CultureInfo.InvariantCulture) + delimiter);


                myStringBuilderRight.Append(rotationRight[i].x.ToString(CultureInfo.InvariantCulture) + delimiter);
                myStringBuilderRight.Append(rotationRight[i].y.ToString(CultureInfo.InvariantCulture) + delimiter);
                myStringBuilderRight.Append(rotationRight[i].z.ToString(CultureInfo.InvariantCulture) + delimiter);
 
        }

        myStringBuilderLeft.Append(rotationHead.x.ToString(CultureInfo.InvariantCulture) + delimiter);
        myStringBuilderLeft.Append(rotationHead.y.ToString(CultureInfo.InvariantCulture) + delimiter);
        myStringBuilderLeft.Append(rotationHead.z.ToString(CultureInfo.InvariantCulture) + delimiter);
 

        myStringBuilderLeft.Append(walkDirection.x.ToString(CultureInfo.InvariantCulture) + delimiter);
        myStringBuilderLeft.Append(walkDirection.y.ToString(CultureInfo.InvariantCulture) + delimiter);
        myStringBuilderLeft.Append(crouchValue.ToString(CultureInfo.InvariantCulture) + delimiter);
        myStringBuilderLeft.Append(rightGrip.ToString(CultureInfo.InvariantCulture) + delimiter);
        myStringBuilderLeft.Append(leftGrip.ToString(CultureInfo.InvariantCulture) + delimiter);

        string stringToSend = myStringBuilderRight.ToString() + myStringBuilderLeft.ToString();

        int mode = canvas.GetMode();

        switch (mode)
        {
            //manual control mode, just sends movement data
            case 0:

                stringToSend = "0" + delimiter + "0" + delimiter + stringToSend;
                SendMessageToStream(stringToSend);
                break;

            //recording mode, sends movement data along with recording name
            case 3:

                stringToSend = "3" + delimiter + canvas.GetCurrentRegistration() + delimiter + stringToSend;
                SendMessageToStream(stringToSend);
                break;
            
            //play mode, sends the name of the recording to play
            case 4:

                stringToSend = "4" + delimiter + canvas.GetCurrentRegistration() + delimiter;
                SendMessageToStream(stringToSend);
                break;

            //does nothing
            default:
                break;
        }
    }

    void OnDestroy()
    {
        if(client != null)
            client.Close();
    }

    public void SendMessageToStream(string message)
    {
        if (client != null)
        {
            message = message + "@";
            bytesToSend = ASCIIEncoding.ASCII.GetBytes(message.ToCharArray());
            nwStream.Write(bytesToSend, 0, bytesToSend.Length);
        }
    }


    public string GetMessageFromStream()
    {
        if (client != null)
        {
            byte[] bytes = new byte[client.ReceiveBufferSize];

            // Read can return anything from 0 to numBytesToRead.
            // This method blocks until at least one byte is read.
            nwStream.Read(bytes, 0, (int)client.ReceiveBufferSize);

            // Returns the data received from the host to the console.
            string returnData = Encoding.UTF8.GetString(bytes);

            return returnData;
        }

        else return "null client";
    }
}
