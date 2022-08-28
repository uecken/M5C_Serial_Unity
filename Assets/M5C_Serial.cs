using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using System.IO.Ports;

public class M5C_Serial : MonoBehaviour
{
    public Transform m5stickC1,m5stickC2;
    public InputField InputField;
    private SerialPort serialPort;
    public string portName = "COM22";
    public int baudRate = 115200;
    //private string rotate_way = "QUATERNION";
    private Quaternion base_q = Quaternion.identity;
    private Quaternion q;
    // Start is called before the first frame update
    void Start()
    {
        // SerialPortの第1引数はArduinoIDEで設定したシリアルポートを設定
        // ArduinoIDEの右下から確認できる
        serialPort = new SerialPort(portName, baudRate, Parity.None, 8, StopBits.One);
        //serialPort = new SerialPort("/dev/cu.usbmode...", 115200); // ここを自分の設定したシリアルポート名に変えること
        serialPort.Open();
        //inputField = GameObject.Find("InputField").GetComponent<InputField>();
    }

    public void ButtonBaseOffset(){
        Debug.Log ("You have clicked the button!");
        base_q = q;
    }

    public void ButtonSerialConnect(){
        if(serialPort.IsOpen){
            serialPort.Close();
            
        }
        if(!serialPort.IsOpen){
            //int portName = int.Parse(InputField.text);
            string portName = InputField.text;
            Debug.Log (portName);
            serialPort = new SerialPort(portName, baudRate, Parity.None, 8, StopBits.One);
            serialPort.Open();                    
        }
            
    }

    // Update is called once per frame
    void Update()
    {
        Debug.Log(Time.deltaTime);
        if (serialPort.IsOpen)
        {
            string data = serialPort.ReadLine();
            //Debug.Log(data);
            string[] sensors = data.Split(',');
            float pitch = float.Parse(sensors[6]);
            float roll = float.Parse(sensors[7]);
            float yaw = float.Parse(sensors[8]);
            float z_rotate = pitch;  
            float x_rotate = roll;

            /***===Rotate M5StickC1 by Quaternion===***/
            float[] q_array = new float[4];
            q_array[0] = float.Parse(sensors[9]);
            q_array[1] = float.Parse(sensors[10]);
            q_array[2] = float.Parse(sensors[11]);
            q_array[3] = float.Parse(sensors[12]);
            q = new Quaternion(q_array[1], q_array[3], q_array[2], -q_array[0]).normalized;
            m5stickC1.transform.rotation = Quaternion.Inverse(base_q) * q;
            Debug.Log(q);

            /***===Rotate M5StickC2 by Euler===***/
            Quaternion q_euler = Quaternion.Euler(new Vector3(-roll,yaw,-pitch)); //-roo,yaw,-pitchにより右手系から左手系へ変換
            m5stickC2.transform.rotation = q_euler;



            /*
            Quaternion targetRot;
            targetRot = Quaternion.Euler(roll-this.transform.localEulerAngles.x,yaw-this.transform.localEulerAngles.y,pitch-this.transform.localEulerAngles.z);
            this.transform.rotation = Quaternion.Slerp(transform.rotation, targetRot, Time.deltaTime);
            */
            /*
            var q = Quaternion.Euler(new Vector3(roll,yaw,pitch));
            var p = Quaternion.Euler(this.transform.localEulerAngles);
            var requireRot = q * Quaternion.Inverse(p);
            this.transform.rotation  = requireRot * this.transform.rotation;
            */
        }
    }
}