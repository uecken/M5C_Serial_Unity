using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using System.IO.Ports;
using TMPro;
using System.Threading;

public class M5C_Serial : MonoBehaviour
{
    public Transform m5stickC1,m5stickC2;
    public InputField PortInputField;
    private SerialPort serialPort;
    public string portName = "COM3";
    public int baudRate = 115200;


    public InputField serialInputField; // シリアル入力用のInputField
    public Text serialOutputText;       // シリアル出力を表示するText
    public Text RollText;       // シリアル出力を表示するText
    public Text PitchText;       // シリアル出力を表示するText
    public Text YawText;       // シリアル出力を表示するText
    public Text RollGraphText;       // シリアル出力を表示するText
    public Text PitchGraphText;       // シリアル出力を表示するText
    public Text YawGraphText;       // シリアル出力を表示するText
    public Button connectButton;       // シリアル接続用のボタン


    private Thread serialThread;
    private Queue<string[]> plotQueue = new Queue<string[]>();
    private object queueLock = new object();

    private Queue<string[]> msgQueue = new Queue<string[]>();
    private object msgqueueLock = new object();

    private bool LEFT_DISPLAY = true; //左ディスプレイ用の設定(右手持ち用)


    //private float pitch = 0;
    //private float roll = 0;
    //private float yaw = 0;
    //private string[] sensors = new string[13];
    //private string rotate_way = "QUATERNION";
    private Quaternion base_q = Quaternion.identity;
    private Quaternion q;



    public GameObject pointPrefab; // グラフの点として使用するプレハブ
    //public GameObject graphArea;   // 描画エリアとして使用するオブジェクト
    private List<GameObject> points = new List<GameObject>(); // 描画された点を保持するリスト
    private List<GameObject> points_pk = new List<GameObject>(); //player keyに対応するroll/pitch,msgを保持するリスト
    private List<GameObject> points_msg = new List<GameObject>(); //player keyに対応するmsgを保持するリスト

    // グラフのスケールを決定するための変数
    //public GameObject graphArea;   // 描画エリアとして使用するオブジェクト

    // グラフのスケールを決定するための変数
    private float graphWidth;  // グラフの幅
    private float graphHeight; // グラフの高さ
    
    /*
    public Canvas canvas; // Canvasオブジェクトへの参照
    public GameObject textPrefab; // 角度の表示に使用するプレハブ
    public Material lineMaterial;  // 線のマテリアル
    */
    public Canvas canvas; // Canvasオブジェクトへの参照
    public RectTransform canvasChild;
    public GameObject linePrefab; // 軸線用のImageプレハブ
    public GameObject textPrefab; // 角度表示用のTextプレハブ
    public GameObject messagePrefab; // msg表示用のTextプレハブ. buildして実行するとnullになるので、start()で生成
    int x_offset = 0;
    float pitch,roll,yaw;


    Color orange = new Color(1f, 0.64f, 0f);
    Color[] colors = new Color[] {
        Color.green, // 緑
        Color.yellow, // 黄色
        //new Color(0.5f, 0f, 0.5f) // 紫 (RGB)
        new Color(1f, 0.64f, 0f),
        Color.cyan, // シアン
        Color.magenta, // マゼンタ
        //Color.gray, // グレー
        //Color.white // 白
        //Color.black, // 黒
    };

    //public GameObject linePrefab; // 軸線用のプレハブ
    //public GameObject textMeshProPrefab; // 角度表示用のTextMeshProプレハブ
    //public GameObject plane;      // 描画対象のPlane

    // Start is called before the first frame update
    void Start()
    {
        // SerialPortの第1引数はArduinoIDEで設定したシリアルポートを設定
        // ArduinoIDEの右下から確認できる
        /*
        serialPort = new SerialPort(portName, baudRate, Parity.None, 8, StopBits.One);
        //serialPort = new SerialPort("/dev/cu.usbmode...", 115200); // ここを自分の設定したシリアルポート名に変えること
        serialPort.Open();
        //inputField = GameObject.Find("InputField").GetComponent<InputField>();
        */

        //serialThread = new Thread(ReadSerial);
        //serialThread.Start();
        PortInputField.text = portName;
        ButtonSerialConnect();
        //delayAndSendSerial(0.5f,"SHOW_REFERENCES");

        //オイラー表示のOFF
        ToggleM5StickC2Visibility();

        // canvasChildオブジェクトのサイズを、canvasの特定の割合（例えば40%）に設定し、整数に丸める
        RectTransform canvasRect = canvas.GetComponent<RectTransform>();
        float newWidth = Mathf.RoundToInt(canvasRect.sizeDelta.x * 0.4f);
        float newHeight = Mathf.RoundToInt(canvasRect.sizeDelta.y * 0.4f);
        canvasChild.sizeDelta = new Vector2(newWidth, newHeight);
        canvasChild.anchoredPosition = Vector2.zero; // canvasの中央に設置

        DrawAxis();

        // ResourcesフォルダからPrefabをロード
        messagePrefab = Resources.Load<GameObject>("GraphMsgTex");
        if (messagePrefab != null)
        {
            Debug.Log("msgPrefab is loaded.");
        }
        else
        {
            Debug.LogError("msgPrefab is not loaded.");
        }

        //serialInputField.onEndEdit.AddListener(delegate { SendSerialInput(serialInputField.text); });

        

        /*
        // 描画オブジェクトの設定
        if (graphArea != null)
        {
            BoxCollider collider = graphArea.GetComponent<BoxCollider>();
            if (collider != null)
            {
                graphWidth = collider.size.x;
                graphHeight = collider.size.y;
                
                //DrawAxisOnPlane();
            }
            else
            {
                // オプション: Colliderが見つからない場合のエラーメッセージ
                Debug.LogError("Graph AreaにBoxColliderが見つかりません。");
            }
        }else
        {
            // オプション: graphAreaが設定されていない場合のエラーメッセージ
            Debug.LogError("Graph Areaが設定されていません。");
        }
        */
    }

    // シリアル入力の送信
    public void SendSerialInput(string input)
    {
        if (serialPort != null && serialPort.IsOpen)
        {

            if(input == ""){
                input = serialInputField.text;
                serialPort.WriteLine(input);
                Debug.Log("SendSerial:"+input);
            }else{
                serialPort.WriteLine(input);
                Debug.Log("SendSerial:"+input);
            }

            //入力後にInputFieldをクリア
            serialInputField.text = "";

        }
    }




    public void ButtonBaseOffset(){
        Debug.Log ("You have clicked the button!");
        base_q = q;
    }

    public void ButtonSerialConnect()
    {
        // すでに開いている場合は閉じる
        if (serialPort != null && serialPort.IsOpen)
        {
            serialPort.Close();
            Debug.Log("Serial Port Closed");
        }

        // 新しいCOMポートで開く
        string portNameInput = PortInputField.text;
        serialPort = new SerialPort(portNameInput, baudRate, Parity.None, 8, StopBits.One);

        try
        {
            serialPort.Open();
            Debug.Log("Serial Port Opened on " + portNameInput);
            serialThread = new Thread(ReadSerial);
            serialThread.Start();
            SendSerialInput("UNITY,START");

            //連続入力だと後段の入力が受け付けられない
            //SendSerialInput("SERIAL,ON");
            //SendSerialInput("SHOW_REFERENCES);
        }
        catch (System.Exception ex)
        {
            Debug.LogError("Failed to open serial port: " + ex.Message);
        }
    }

    void ReadSerial()
    {
        while (true)
        {
            try
            {
                if (serialPort == null || !serialPort.IsOpen)
                {
                    // シリアルポートが閉じている場合の処理z
                    // 例: 再接続の試み、ループの終了、等
                    Debug.LogWarning("シリアルポートが閉じています。");
                    continue;
                }
                
                    string data = serialPort.ReadLine();
                    Debug.Log(data);
                    // 必要に応じてUIの更新処理をここに追加
                    // 例: this.Invoke(() => { serialOutputText.text = data; });

                    //Debug.Log(data);
                    //serialOutputText.text = data; // シリアル出力をテキストオブジェクトに表示

                    string[] sensors = data.Split(',');
                    if(sensors[0] == "pk_references" || sensors[0] == "MOTION_MESSAGE"){
                        Debug.Log(sensors[0]);
                        //drawPitchRollVectol(sensors);
                        lock (queueLock)
                        {
                            plotQueue.Enqueue(sensors);
                        }
                    }else if(sensors[0] == "sensor_data"){
                        Debug.Log("sensor_data");
                        lock (queueLock)
                        {
                            plotQueue.Enqueue(sensors);
                        }
                    }else if(sensors[0] == "selected_pk" || sensors[0] == "MOTION_MESSAGE_selected"){
                        //if(DEBUG_HID)Serial.printf("selected_pk,%c,%f,%f\r\n",pk_selected.hid_input,pk_selected.rpy[0]);
                        Debug.Log(sensors[0]);
                        lock (queueLock)
                        {
                            plotQueue.Enqueue(sensors);
                        }
                    }else if(sensors[0] == "LEFT_DISPLAY"){
                        
                    }else if(sensors[0] == "Init Unity Roll and Pitch"){
                        Debug.Log(sensors[0]);
                        lock (queueLock)
                        {
                            plotQueue.Enqueue(sensors);
                        }
                    }
            }
            catch (System.Exception ex){
                Debug.LogError("ReadSerial Catch:"+ex.Message);
                break;
            }
        }
    }

    void OnDestroy()
    {
        if (serialThread != null && serialThread.IsAlive)
        {
            serialThread.Abort();
        }
        if (serialPort != null && serialPort.IsOpen)
        {
            serialPort.Close();
        }
    }

    // Update is called once per frame
    void Update()
    {

        if (plotQueue.Count > 0)
        {
            string[] sensors;
            lock (queueLock)
            {
                sensors = plotQueue.Dequeue();
            }

            //serialOutputText.text = sensors;]
            serialOutputText.text = string.Join(",", sensors);

            if(sensors[0] == "Init Unity Roll and Pitch"){
                //描画されたpoints_pkとpoints_msgの削除
                for (int i = 0; i < points_pk.Count; i++)
                {
                    Destroy(points_pk[i]);
                }
                for (int i = 0; i < points_msg.Count; i++)
                {
                    Destroy(points_msg[i]);
                }
                points_pk.Clear();
                points_msg.Clear();
            }
            else if(sensors[0] == "pk_references" ||sensors[0] == "MOTION_MESSAGE"){
                //マイコンに保存された基準姿勢を表示
                Debug.Log("drawing:"+sensors[0]);
                drawPitchRollVectol(sensors[0],sensors, Color.blue);
            }
            else if(sensors[0] == "sensor_data"){
                //現在の姿勢を表示

                /***===Rotate M5StickC1 by Quaternion===***/
                float[] q_array = new float[4];
                q_array[0] = float.Parse(sensors[10]);
                q_array[1] = float.Parse(sensors[11]);
                q_array[2] = float.Parse(sensors[12]);
                q_array[3] = float.Parse(sensors[13]);
                q = new Quaternion(q_array[1], q_array[3], q_array[2], -q_array[0]).normalized;
                m5stickC1.transform.rotation = Quaternion.Inverse(base_q) * q;
                Debug.Log(q);

                /***===Rotate M5StickC2 by Quaternion yaw offset===***/

                pitch = float.Parse(sensors[7]);
                roll = float.Parse(sensors[8]);
                yaw = float.Parse(sensors[9]);
                float z_rotate = pitch;
                float x_rotate = roll;
                AddPointToGraph();


                /***===Rotate M5StickC2 by Euler(Z-X-Y回転)===***/
                
                //Quaternion q_euler = Quaternion.Euler(new Vector3(-roll,yaw,-pitch)); //-roo,yaw,-pitchにより右手系から左手系へ変換
                //回転順がZXYのため、roll方向の表示がおかしい。特にroll 70~90度以外の角度。
                //m5stickC2.transform.rotation = q_euler;
                


                /***===Rotate M5StickC2 by Euler(X-Y-Z回転)===***/
                // 個々の軸の回転をクォータニオンとして生成
                
                
                Quaternion rotationX = Quaternion.Euler(-roll, 0, 0);
                Quaternion rotationY = Quaternion.Euler(0, pitch, 0);
                //Quaternion rotationZ = Quaternion.Euler(0, 0, yaw);
                //Quaternion totalRotation = rotationX * rotationY * rotationZ; // X-Y-Z順に回転を適用
                Quaternion totalRotation = rotationX * rotationY;  // X-Y順に回転を適用 (Yawは正しくないため無視)
                m5stickC2.transform.rotation = totalRotation; //Pitch 70-90度でジンバルロックによりrollが異常
                
                





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
            }else if(sensors[0] == "selected_pk" || sensors[0] == "MOTION_MESSAGE_selected"){
                //選択された姿勢を表示
                Debug.Log(sensors[0]);
                //Color randomColor = new Color(Random.Range(0f, 1f), Random.Range(0f, 1f), Random.Range(0f, 1f));
                //Color randomColor = colors[Random.Range(0, colors.Length)];
                drawPitchRollVectol(sensors[0],sensors, orange);
            }
        }
    }

    void DrawAxis()
    {
        RectTransform canvasRect = canvasChild.GetComponent<RectTransform>();
        float canvasWidth = canvasRect.sizeDelta.x;
        float canvasHeight = canvasRect.sizeDelta.y;
        int x_text_offset = 70;
        int y_text_offset = -20;

        // 中心の0°から両端の-90°と90°まで30°刻みで軸線とテキストを描画
        // 縦軸の軸線とテキストを描画
        for (int i = -180; i <= 180; i += 30)
        {
            float x = Map(i, -180, 180, -canvasWidth / 2, canvasWidth / 2);

            // 縦軸線の描画
            CreateAxisLine(x+x_offset, 0, 1, canvasHeight);

            // 縦軸テキストの表示
            if(i%90==0)CreateAxisText(x+x_offset+x_text_offset, y_text_offset, i.ToString() + "°");
        }

        // 横軸の軸線とテキストを描画
        for (int j = -90; j <= 90; j += 30)
        {
            float y = Map(j, -90, 90, -canvasHeight / 2, canvasHeight / 2);
            //float y = Map(j, -90, 90, -canvasHeight , canvasHeight / 2);

            // 横軸線の描画
            CreateAxisLine(x_offset, y, canvasWidth, 1);

            // 横軸テキストの表示
            if(j%90==0 && j!=0)CreateAxisText(x_offset+20+45, y, j.ToString() + "°");
        }
    }


    void CreateAxisLine(float x, float y, float width, float height)
    {
        GameObject lineObject = Instantiate(linePrefab, canvasChild.transform);
        RectTransform lineRect = lineObject.GetComponent<RectTransform>();
        lineRect.anchoredPosition = new Vector2(x, y);
        lineRect.sizeDelta = new Vector2(width, height);
    }

    void CreateAxisText(float x, float y, string text)
    {
        GameObject textObject = Instantiate(textPrefab, canvasChild.transform);
        RectTransform textRect = textObject.GetComponent<RectTransform>();
        textRect.anchoredPosition = new Vector2(x, y);
        Text textComponent = textObject.GetComponent<Text>();
        textComponent.text = text;
    }

/*
    void DrawAxisOnPlane(){
        // Planeのサイズを取得
        Vector3 planeSize = plane.GetComponent<Renderer>().bounds.size;

        // 中心の0°から両端の-180°と180°まで30°刻みで軸線とテキストを描画
        for (int i = -180; i <= 180; i += 30)
        {
            float x = Map(i, -180, 180, -planeSize.x / 2, planeSize.x / 2);

            // 軸線の描画
            GameObject lineObject = Instantiate(linePrefab, plane.transform);
            lineObject.transform.localPosition = new Vector3(x, 0, 0);
            lineObject.transform.localScale = new Vector3(0.01f, planeSize.y, 0.01f); // 薄い線

            // 角度の表示
            GameObject textObject = Instantiate(textMeshProPrefab, plane.transform);
            textObject.transform.localPosition = new Vector3(x, -0.2f, 0); // テキストの位置
            TextMeshPro textMesh = textObject.GetComponent<TextMeshPro>();
            textMesh.text = i + "°";
        }
    }
    */

    public void ToggleM5StickC2Visibility()
    {
        m5stickC2.gameObject.SetActive(!m5stickC2.gameObject.activeSelf);
    }


void AddPointToGraph()
{
    //M5StickCから取得したRoll/Pitchの表示
    RollText.text = roll.ToString();
    PitchText.text = pitch.ToString();
    //YawText.text = yaw.ToString();


/*
    RectTransform canvasRect = canvasChild.GetComponent<RectTransform>();
    float canvasWidth = canvasRect.sizeDelta.x;
    float canvasHeight = canvasRect.sizeDelta.y;
*/

    RectTransform canvasChildRect = canvasChild.GetComponent<RectTransform>();
    float canvasChildWidth = canvasChildRect.sizeDelta.x;
    float canvasChildHeight = canvasChildRect.sizeDelta.y;

    GameObject newPoint = Instantiate(pointPrefab, canvasChild.transform);
    RectTransform pointRect = newPoint.GetComponent<RectTransform>();

    // Rendererの代わりにImageコンポーネントを使用
    Image pointImage = newPoint.GetComponent<Image>();
    if (pointImage != null)
    {
        // ピッチが80度以上または-80度以下の場合、点を灰色に設定
        if (pitch >= 80 || pitch <= -80)
        {
            pointImage.color = new Color(0.5f, 0.5f, 0.5f, 1f); // 灰色
        }
        else
        {
            // 他の条件（赤、緑など）に基づいて色を設定
            pointImage.color = Color.red; // ここで設定した色に変更
        }
    }
    /*
    if (pointImage != null)
    {
        if (roll < 0) 
        {
            pointImage.color = new Color(0.5f, 0.5f, 0.5f, 1f); // 灰色
            roll = roll * -1;
        }
        else
        {
            pointImage.color = new Color(1f, 0f, 0f, 1f); // 赤色
        }
    }
    */

    // roll=90° or -90°を中心にするための前処理
    roll = rollOffset(roll);

    RollGraphText.text = roll.ToString();
    PitchGraphText.text = pitch.ToString();

    //float x = Map(roll, -180, 180, -canvasWidth / 2, canvasWidth / 2);
    //float y = Map(pitch, -90, 90, -canvasHeight / 2, canvasHeight / 2);
    float x = Map(roll, -180, 180, -canvasChildWidth / 2, canvasChildWidth / 2) + x_offset;
    float y = Map(pitch, -90, 90, -canvasChildHeight / 2, canvasChildHeight / 2);

    pointRect.anchoredPosition = new Vector2(x, y);


    points.Add(newPoint);

    // 古い点を削除
    if (points.Count > 2) // ここで保持する点の数を設定
    {
        Destroy(points[0]);
        points.RemoveAt(0);
    }
}


    // 値を一つの範囲から別の範囲にマッピングするメソッド
    float Map(float value, float fromSource, float toSource, float fromTarget, float toTarget)
    {
        return (value - fromSource) / (toSource - fromSource) * (toTarget - fromTarget) + fromTarget;
    }


    float rollOffset(float roll){
        //rollをオフセット
        if (!LEFT_DISPLAY && roll < -90) 
        {
            roll = 180 + (-1 * (-180 - roll));
        }
        else if (LEFT_DISPLAY && roll > 90) {
            roll = -180 + (roll - 180);
        }

        // roll=90°を中心にするためのオフセット
        if(!LEFT_DISPLAY) roll = roll - 90;
        else if(LEFT_DISPLAY) roll = roll +90;

        return roll;
    }


    void drawPitchRollVectol(string msg, string[] sensors, Color pointColor){
        float roll=0;
        float pitch=0;
        float yaw=0;
        char input;
        string inputs_str = "";

        //roll,pitchを取得
        if(msg=="pk_references"){
            roll = float.Parse(sensors[8]); //8?
            pitch = float.Parse(sensors[9]); //9?
            yaw = float.Parse(sensors[10]);
            //input = sensors[14];
            char [] inputs = sensors[16].ToCharArray();
            //inputsを文字列で出力
            for(int i=0;i<inputs.Length;i++){
                inputs_str += inputs[i];
            }
            Debug.Log("input:"+inputs_str);
        }else if(msg=="selected_pk"){
            //selected_pk,?,roll,pitch
            roll = float.Parse(sensors[1]); //3?
            pitch = float.Parse(sensors[2]); //4?
            yaw = float.Parse(sensors[3]); // 4.26 not used
        }else if(msg=="MOTION_MESSAGE" || msg=="MOTION_MESSAGE_selected"){
            //Serial.printf("message_vector,%s,%d,%d,%d \r\n",
            roll = float.Parse(sensors[2]);
            pitch = float.Parse(sensors[3]);
            yaw = float.Parse(sensors[4]);
            inputs_str = sensors[1];
            Debug.Log("motion_msg:"+inputs_str);
        }
        

        //=====点の描画=====
        roll = rollOffset(roll);
        
        //roll,pitchをx軸,y軸に変換
        float x = Map(roll, -180, 180, -canvasChild.sizeDelta.x / 2, canvasChild.sizeDelta.x / 2) + x_offset;
        float y = Map(pitch, -90, 90, -canvasChild.sizeDelta.y / 2, canvasChild.sizeDelta.y / 2);

        //描画
        GameObject newPoint = Instantiate(pointPrefab, canvasChild.transform);
        RectTransform pointRect = newPoint.GetComponent<RectTransform>();
        pointRect.anchoredPosition = new Vector2(x, y);
    
        // Imageコンポーネントに色を設定
        Image pointImage = newPoint.GetComponent<Image>();
        if (pointImage != null)
        {
            pointImage.color = pointColor;
        }

        
        //=====後処理=====
        if (msg == "selected_pk" || msg == "MOTION_MESSAGE_selected")
        { // msg が "selected_pk" の場合、0.5秒後に色を青に変更
            StartCoroutine(ChangeColorAfterDelay(newPoint, 0.15f, Color.blue));
        }else if(msg == "pk_references" || msg =="MOTION_MESSAGE"){
            //===文字列の描画===
            //点の位置に文字列(inputs)を、表示する.色はcolors配列からランダムに選択する
            Debug.Log("Adding Motion_msg:"+inputs_str);
            if (messagePrefab == null) {
                Debug.LogError("msgPrefab is not assigned.");
            }else{
                GameObject textObject = Instantiate(messagePrefab, canvasChild.transform);
                Debug.Log("1");
                RectTransform textRect = textObject.GetComponent<RectTransform>();
                Debug.Log("2");
                //textRect.anchoredPosition = new Vector2(x+143, y+5);
                textRect.anchoredPosition = new Vector2(x+170, y+5);
                Debug.Log("3");
                Text textComponent = textObject.GetComponent<Text>(); 
                Debug.Log("4");
                textComponent.text = inputs_str;
                Debug.Log("5");
                textComponent.color = colors[Random.Range(0, colors.Length)];;
                Debug.Log("6");

                //textオブジェクトを保存
                points_msg.Add(textObject);
            }

        }

        points_pk.Add(newPoint);

        /*
        Serial.println("%u,%u,%c,%d,%d,%d,%d,%d,%d,%d,%f,%f,%f,%f,%c,%c%c%c%c%c%c%c%c,%c,%u",
            pk_vector[i].mode,
            pk_vector[i].skill_priority,
            pk_vector[i].rpy_priority,
            pk_vector[i].roll_min,
            pk_vector[i].roll_max,
            pk_vector[i].pitch_min,
            pk_vector[i].pitch_max,
            pk_vector[i].rpy[0], pk_vector[i].rpy[1], pk_vector[i].rpy[2],
            pk_vector[i].quatanion[0], pk_vector[i].quatanion[1], pk_vector[i].quatanion[2], pk_vector[i].quatanion[3],
            pk_vector[i].hid_input,
            pk_vector[i].hid_inputs[0], pk_vector[i].hid_inputs[1], pk_vector[i].hid_inputs[2], pk_vector[i].hid_inputs[3],
            pk_vector[i].hid_inputs[4], pk_vector[i].hid_inputs[5], pk_vector[i].hid_inputs[6], pk_vector[i].hid_inputs[7],
            pk_vector[i].hid_input_interval,
            pk_vector[i].hid_input_acc_threshold);
        */
    }   

    IEnumerator ChangeColorAfterDelay(GameObject obj, float delay, Color newColor)
    {
        // 指定された時間待機
        yield return new WaitForSeconds(delay);

        // 色を変更
        Image pointImage = obj.GetComponent<Image>();
        if (pointImage != null)
        {
            pointImage.color = newColor;
        }
    }

    void UpdateButtonText(Button button, string text)
    {
        Text buttonText = button.GetComponentInChildren<Text>();
        if (buttonText != null)
        {
            buttonText.text = text;
        }
    }


    IEnumerator delayAndSendSerial(float delay,string msg)
    {
        // ここで3秒間待機
        yield return new WaitForSeconds(delay);

        // 3秒後に実行される処理
        SendSerialInput(msg);
        Debug.Log("3 seconds have passed!");
    }

}