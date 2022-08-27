using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Button : MonoBehaviour
{
    public M5C_Serial M5C;
    public void OnClick(){
        Debug.Log("Button Pushed");
        M5C.ButtonOnClick();
    }
}
