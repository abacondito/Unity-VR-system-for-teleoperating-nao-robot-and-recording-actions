using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class ButtonListButton : MonoBehaviour
{
    public Text myText;

    private string myString;

    public ZoraMenuControl canvas;

    public void setText(string textString)
    {
        myText.text = textString;
        myString = textString;
    }

    public void OnClick()
    {
        string message = "4:" + myString.ToString();

        canvas.SetMode(4);

        canvas.SetCurrentRegistration(myString);

        Debug.Log(message);
    }
}
