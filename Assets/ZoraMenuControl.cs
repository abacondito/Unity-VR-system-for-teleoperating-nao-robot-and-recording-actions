using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class ZoraMenuControl : MonoBehaviour
{

    public SocketGatherer socket;

    /*int representing the current mode of operation:
        there are 5:
        0 -> manual controls, zora mimics the hands movement and can walk e lower/raise up
        1 -> default pose, zora uses a default pose and does not mimic hands movement
        2 -> in recordPoseMenu, but not recording
        3 -> in savedPosesMenu
        4 -> recording the hands movement, registration name saved in currentRegistration*/
    public int modeSelected = 0;

    //contains name of the current or last registration, use it only when in registration mode(4) to obtain the registration name
    public string currentRegistration = "";



    public GameObject mainControlMenuUI, defaultPoseMenuUI, savedPosesMenuUI, zoraCommandsImageUI, recordPoseMenuUI;

    public Transform scrollContent;

    public GameObject buttonTemplate;

    public InputField recordNameInputField;

    public Text insertPose, emptyNameErrorMessage, alreadyTakenNameErrorMessage, nowRecordingMessage;

    public GameObject confirmRecordButton;

    void Start()
    {
        zoraCommandsImageUI.SetActive(true);
    }

    public void LoadManualControlsMenu()
    {
        modeSelected = 0;

        zoraCommandsImageUI.SetActive(true);
        defaultPoseMenuUI.SetActive(false);
        recordPoseMenuUI.SetActive(false);
        savedPosesMenuUI.SetActive(false);



        Debug.Log(modeSelected);
    }

    public void LoadDefaultPosesMenu()
    {
        modeSelected = 11;

        //experimental
        socket.SendMessageToStream("11:nan");

        zoraCommandsImageUI.SetActive(false);
        defaultPoseMenuUI.SetActive(true);
        recordPoseMenuUI.SetActive(false);
        savedPosesMenuUI.SetActive(false);

        Debug.Log(modeSelected);
    }

    public void LoadRecordPoseMenu()
    {
        modeSelected = 12;

        //experimental
        socket.SendMessageToStream("12:nan");

        zoraCommandsImageUI.SetActive(false);
        defaultPoseMenuUI.SetActive(false);
        recordPoseMenuUI.SetActive(true);
        savedPosesMenuUI.SetActive(false);

        insertPose.gameObject.SetActive(true);
        recordNameInputField.gameObject.SetActive(true);
        emptyNameErrorMessage.gameObject.SetActive(false);
        alreadyTakenNameErrorMessage.gameObject.SetActive(false);
        confirmRecordButton.SetActive(true);

        recordNameInputField.text = "";

        nowRecordingMessage.gameObject.SetActive(false);

        Debug.Log(modeSelected);
    }

    //function called when the recordedPauseMenuButton is pressed, generates the recorded poses menu
    public void LoadSavedPosesMenu()
    {
        modeSelected = 13;

        //experimental
        socket.SendMessageToStream("13:nan");

        zoraCommandsImageUI.SetActive(false);
        defaultPoseMenuUI.SetActive(false);
        recordPoseMenuUI.SetActive(false);
        savedPosesMenuUI.SetActive(true);

        Debug.Log(modeSelected);

        foreach (Transform child in scrollContent)
        {
            Destroy(child.gameObject);
        }

        socket.SendMessageToStream("2");
        string message = socket.GetMessageFromStream();


        string[] buttons = message.Split(':');

        for(int i=0; i <buttons.Length - 1; i++)
        {
            GameObject button = Instantiate(buttonTemplate) as GameObject;

            button.SetActive(true);

            button.GetComponent<ButtonListButton>().setText(buttons[i]);

            button.transform.SetParent(scrollContent, false);
        }
    }

    public void confirmRecordButtonOnClick()
    {
        bool isEmpty = false;
        bool isAlreadyTaken = false;


        string recordName = recordNameInputField.text.ToString();

        if (recordName == "") isEmpty = true;

        if (isEmpty)
        {
            emptyNameErrorMessage.gameObject.SetActive(true);
            alreadyTakenNameErrorMessage.gameObject.SetActive(false);
        }

        socket.SendMessageToStream("2");
        string message = socket.GetMessageFromStream();

        string[] buttons = message.Split(':');

        for (int i = 0; i < buttons.Length - 1; i++)
        {
            if(recordName == buttons[i])
            {
                isAlreadyTaken = true;
            }
        }

        if (isAlreadyTaken)
        {
            emptyNameErrorMessage.gameObject.SetActive(false);
            alreadyTakenNameErrorMessage.gameObject.SetActive(true);
        }

        if(!isAlreadyTaken && !isEmpty)
        {
            currentRegistration = recordName;
            modeSelected = 3;

            recordNameInputField.gameObject.SetActive(false);
            emptyNameErrorMessage.gameObject.SetActive(false);
            alreadyTakenNameErrorMessage.gameObject.SetActive(false);
            confirmRecordButton.SetActive(false);
            insertPose.gameObject.SetActive(false);

            nowRecordingMessage.gameObject.SetActive(true);
            nowRecordingMessage.text = "Now recording " + recordName + " ..." + "\n\nChange mode to stop it"; 

        }
    }



    public void StandInitPose()
    {
        socket.rightController.resetCrouch();
        socket.SendMessageToStream("1:StandInit");
    }

    public void SitRelaxPose()
    {
        socket.rightController.resetCrouch();
        socket.SendMessageToStream("1:SitRelax");
    }

    public void StandZeroPose()
    {
        socket.rightController.resetCrouch();
        socket.SendMessageToStream("1:StandZero");
    }

    public void LyingBellyPose()
    {
        socket.rightController.resetCrouch();
        socket.SendMessageToStream("1:LyingBelly");
    }

    public void LyingBackPose()
    {
        socket.rightController.resetCrouch();
        socket.SendMessageToStream("1:LyingBack");
    }

    public void StandPose()
    {
        socket.rightController.resetCrouch();
        socket.SendMessageToStream("1:Stand");
    }

    public void CrouchPose()
    {
        socket.rightController.resetCrouch();
        socket.SendMessageToStream("1:Crouch");
    }

    public void SitPose()
    {
        socket.rightController.resetCrouch();
        socket.SendMessageToStream("1:Sit");
    }

    public int GetMode()
    {
        return modeSelected;
    }

    public string GetCurrentRegistration()
    {
        return currentRegistration;
    }


    public void SetMode(int mode)
    {
        modeSelected = mode;
    }

    public void SetCurrentRegistration(string recording)
    {
        currentRegistration = recording;
    }
}
