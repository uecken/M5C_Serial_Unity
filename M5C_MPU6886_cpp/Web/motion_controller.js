document.addEventListener('DOMContentLoaded', () => {
    const uploadSelectedFilesButton = document.getElementById('uploadSelectedFilesButton');
    const webFilesSelect = document.getElementById('webFilesSelect');
    const statusDiv = document.getElementById('status');
    const radioButtons = document.querySelectorAll('input[name="type"]');
    let port;
    let yawOffset = 0;
    let base_q = new THREE.Quaternion();
    let quaternion = new THREE.Quaternion(0,0,0);
    let lastPitch = 0, lastRoll = 0;
    const pitchElem = document.getElementById('pitch');
    const rollElem = document.getElementById('roll');
    const yawElem = document.getElementById('yaw');
    const quaternionElem = document.getElementById('quaternion');
    const pitchRollCanvas = document.getElementById('pitchRollCanvas');
    const ctx = pitchRollCanvas.getContext('2d');
    const bluePoints = [];
    const sphereIntersections = [];

    async function connectSerial() {
        if (!("serial" in navigator)) {
            statusDiv.textContent = "Web Serial API not supported.";
            return null;
        }

        port = await navigator.serial.requestPort();
        await port.open({ baudRate: 115200 });
        readLoop();
    }

    async function readLoop() {
        const reader = port.readable.getReader();
        let readBuffer = '';
        while (true) {
            const { value, done } = await reader.read();
            if (done) {
                reader.releaseLock();
                break;
            }
            const data = new TextDecoder().decode(value);
            readBuffer += data;
            readBuffer = processBuffer(readBuffer);
        }
    }

    function processBuffer(readBuffer) {
        let endIndex;
        while ((endIndex = readBuffer.indexOf('\r\n')) >= 0) {
            const line = readBuffer.slice(0, endIndex);
            readBuffer = readBuffer.slice(endIndex + 2);
            appendSerialOutput(line + '\n');
            handleSerialData(line);
        }
        return readBuffer;
    }

    async function writeToSerial(data) {
        const writer = port.writable.getWriter();
        await writer.write(data);
        writer.releaseLock();
    }

    async function fetchWebFiles() {
        const files = ['firmware/m5c/firmware.bin']; // これをファイルパスに合わせて変更
        webFilesSelect.innerHTML = '';
        files.forEach(file => {
            const option = document.createElement('option');
            option.value = file;
            option.textContent = file;
            webFilesSelect.appendChild(option);
        });
    }

    async function uploadSelectedFiles() {
        const selectedFiles = Array.from(webFilesSelect.selectedOptions).map(option => option.value);
        if (selectedFiles.length === 0) {
            statusDiv.textContent = "No files selected.";
            return;
        }

        if (!port) {
            await connectSerial();
        }

        for (const fileName of selectedFiles) {
            const response = await fetch(`/${fileName}`);
            if (response.ok) {
                const arrayBuffer = await response.arrayBuffer();
                const header = `savepk2vectol,${fileName},${arrayBuffer.byteLength}\n`;

                // Send header
                await writeToSerial(new TextEncoder().encode(header));
                // Send file data
                await writeToSerial(new Uint8Array(arrayBuffer));

                statusDiv.textContent += `Uploaded ${fileName}\n`;
            } else {
                statusDiv.textContent += `Failed to fetch ${fileName} from web storage.\n`;
            }
        }
    }

    function appendSerialOutput(data) {
        const outputDiv = document.createElement('div');
        outputDiv.textContent = data;
        serialOutput.appendChild(outputDiv);
        serialOutput.scrollTop = serialOutput.scrollHeight; // Scroll to bottom
    }

    function handleSerialData(line) {
        if (line.startsWith('sensor_data')) {
            const parts = line.split(',');
            if (parts.length === 14) {
                let pitch = parseFloat(parts[7]);
                let roll = parseFloat(parts[8]);
                let yaw = parseFloat(parts[9]);

                /*
                let qx = parseFloat(parts[10]);
                let qy = parseFloat(parts[11]);
                let qz = parseFloat(parts[12]);
                let qw = parseFloat(parts[13]);
                */

                let qw = parseFloat(parts[10]);
                let qx = parseFloat(parts[11]);
                let qy = parseFloat(parts[12]);
                let qz = parseFloat(parts[13]);

    
                if (document.getElementById('eulerToQuaternion').checked) {
                    const euler = new THREE.Euler(
                        THREE.Math.degToRad(pitch),
                        THREE.Math.degToRad(roll),
                        THREE.Math.degToRad(yaw),
                        'YXZ'
                    );
                    quaternion.setFromEuler(euler);
                    qx = quaternion.x;
                    qy = quaternion.y;
                    qz = quaternion.z;
                    qw = quaternion.w;
                    
                   /*
                    quaternion.setFromEuler(euler);
                    qx = quaternion.y;
                    qy = quaternion.w;
                    qz = quaternion.z;
                    qw = -quaternion.x;
                    */
                }
    
                pitchElem.textContent = pitch.toFixed(3);
                rollElem.textContent = roll.toFixed(3);
                yawElem.textContent = yaw.toFixed(3);
                quaternionElem.textContent = `${qx.toFixed(3)}, ${qy.toFixed(3)}, ${qz.toFixed(3)}, ${qw.toFixed(3)}`;
    
                //quaternion.set(qx, qy, qz, qw);

                //unityの引数はwxyzの順
                //q = new Quaternion(q_array[1], q_array[3], q_array[2], -q_array[0]).normalized;
                //q = new Quaternion(qx, qz, qy, -qw).normalized;
                //m5stickC1.transform.rotation = Quaternion.Inverse(base_q) * q;
                
                //quaternion.set(qy, qw, qz, -qx);
                //quaternion.set(qy, -qw, -qz, -qx);
                //quaternion.set(-qy, qw, -qz, qx);

                //quaternion.set(qy, qw, -qz, qx);

                //quaternion.set(qy, qw, qz, -qx);//yaw回転方向が逆
//                quaternion.set(qy, qw, qz, -qx);
                //quaternion.set(qx, -qy, -qz, qw);
                
    
//                quaternion.set(qx, qz, -qy, qw);  // UnityからThree.jsへの
                
               //var q = new THREE.Quaternion(-qz, qw, -qx, qy);
               //quaternion.set(-qz, qw, -qx, qy);

               //quaternion.set(-qw, -qx, -qy, qz);//roll pitch逆 @YXZ
               //quaternion.set(-qy, qz, qw, qx);//-qw, -qx, -qy, qzと同じ

               //quaternion.set(qx, -qy, -qz, qw);

               //quaternion.set(qw, qx, qy, qz);

                //Quaternion( x : Float, y : Float, z : Float, w : Float )
                //quaternion.set(qx, qy, qz, qw);
                //quaternion.set(-qx, qy, qz, -qw);
               //quaternion.set(-qz, qw, -qx, qy);

               //quaternion.set(qx, qy, qz, qw);
               //quaternion.set(qx, -qy, qz, -qw); //おしい
               //quaternion.set(-qx, -qy, -qz, qw); //おしい
               //quaternion.set(-qx, -qy, -qz, -qw);
               //quaternion.set(-qx, qy, qz, -qw); //おしい


               //quaternion.set(qx, qy, -qz, qw);//カメラ視点が変わればOKと思ったが、pitchが逆。
               //uprightQuaternion.setFromEuler(new THREE.Euler(0, THREE.Math.degToRad(90), 0, 'XYZ'));



               //quaternion.set(qw, qx, qy,qz);
               //quaternion.set(qx, qz, qy,-qw);//unityと同じ
               //quaternion.set(qz, qy, qw,-qx);//unityと同じで引数順調整
               //quaternion.set(qw, qx, qz,-qy);//unityと同じで引数順調整

    


               //quaternion.set(-qw, qx, qz, qy);//unityの順番を変えたので本来正しいはず
               //quaternion.set(qw, qx, qz, -qy);//本来正しいはず
               //quaternion.set(qw, -qx, qz, qy);//本来正しいはず
               //quaternion.set(-qw, qy, qz, qx);//OKかと思ったがOKではない
               //quaternion.set(-qw, qy, qz, qx);//OKかと思ったがOKではない

               //quaternion.set(qy, qw, qz, -qx);//yaw回転方向が逆
            
               //quaternion.set(-qz, qw, -qx, qy)
               //quaternion.set( -qx, qy, qz, -qw )

               //quaternion.set(-qw, qx, qz, qy);//unityの順番を変えたので本来正しいはず
               //quaternion.set(qw, -qx, qz, qy);//unityの順番を変えたので本来正しいはず
               //quaternion.set(-qz, qy, qw, qx);//https://ovide.hatenablog.com/entry/2018/05/16/183232

                //unityの引数はxyzzの順
                //q = new Quaternion(q_array[1], q_array[3], q_array[2], -q_array[0]).normalized;
                //q = new Quaternion(qx, qz, qy, -qw).normalized;
                //さらにxとyの符号を逆にすればよい。
                //https://stackoverflow.com/questions/18066581/convert-unity-transforms-to-three-js-rotations
                quaternion.set(-qx, qz, qy, qw)



                //quaternion.set( qx, -qy, -qz, qw ) https://techblog.kayac.com/2022-group-calendar-three-js-sync-to-unity-webgl
                //quaternion.set( -qx, qy, qz, -qw ) 
                //quaternion.set(qw, qx, -qz, -qy);//本来正しいはず



                updatePitchRollCanvas(pitch, roll);
    
    
                lastPitch = pitch;
                lastRoll = roll;
            }
        } else if (line.startsWith('selected_pk')) {
            plotPoint(lastPitch, lastRoll, 'blue');
            plotIntersectionWithSphere(quaternion);
        }
    }

    function updatePitchRollCanvas(pitch, roll) {
        ctx.clearRect(0, 0, pitchRollCanvas.width, pitchRollCanvas.height);
        drawAxis(ctx); // Draw the axis again
        ctx.beginPath();
        ctx.arc(pitchRollCanvas.width / 2, pitchRollCanvas.height / 2, 5, 0, 2 * Math.PI);
        ctx.fill();
        bluePoints.forEach(point => {
            ctx.beginPath();
            ctx.arc(point.x, point.y, 5, 0, 2 * Math.PI);
            ctx.fillStyle = 'blue';
            ctx.fill();
        });
        const x = (roll + 180) * (pitchRollCanvas.width / 360);
        const y = (pitch + 90) * (pitchRollCanvas.height / 180);
        ctx.beginPath();
        ctx.arc(x, y, 5, 0, 2 * Math.PI);
        ctx.fillStyle = 'red';
        ctx.fill();
    }

    function plotPoint(pitch, roll, color) {
        const x = (roll + 180) * (pitchRollCanvas.width / 360);
        const y = (pitch + 90) * (pitchRollCanvas.height / 180);
        bluePoints.push({ x, y, color });
    }

    function plotIntersectionWithSphere(quaternion) {
        const direction = new THREE.Vector3(0, 0, 1);
        direction.applyQuaternion(quaternion).normalize();
        const intersection = direction.clone().multiplyScalar(1);
        const color = intersection.z >= 0 ? 0x0000ff : 0x800080;

        const geometry = new THREE.SphereGeometry(0.05, 32, 32);
        const material = new THREE.MeshBasicMaterial({ color: color });
        const sphere = new THREE.Mesh(geometry, material);

        sphere.position.copy(intersection);
        sphere.scale.setScalar(1 / camera.position.distanceTo(sphere.position));

        scene.add(sphere);
        sphereIntersections.push(sphere);
    }

    function drawAxis(ctx) {
        ctx.save();
        ctx.globalAlpha = 0.5;
        ctx.clearRect(0, 0, pitchRollCanvas.width, pitchRollCanvas.height);
        ctx.beginPath();
        ctx.strokeStyle = '#000000';

        for (let i = 0; i <= 360; i += 30) {
            ctx.moveTo(i * (pitchRollCanvas.width / 360), 0);
            ctx.lineTo(i * (pitchRollCanvas.width / 360), pitchRollCanvas.height);
            ctx.fillText(i - 180, i * (pitchRollCanvas.width / 360), pitchRollCanvas.height / 2 + 10);
        }

        for (let i = 0; i <= 180; i += 30) {
            ctx.moveTo(0, i * (pitchRollCanvas.height / 180));
            ctx.lineTo(pitchRollCanvas.width, i * (pitchRollCanvas.height / 180));
            ctx.fillText(i - 90, pitchRollCanvas.width / 2 + 10, i * (pitchRollCanvas.height / 180));
        }

        ctx.stroke();
        ctx.restore();
    }

    document.getElementById('connectBtn').addEventListener('click', async () => {
        await connectSerial();
    });

    document.getElementById('sendBtn').addEventListener('click', async () => {
        const input = document.getElementById('serialInput').value;
        if (port && input) {
            await writeToSerial(new TextEncoder().encode(input + '\n'));
        }
    });

    document.getElementById('initYawBtn').addEventListener('click', () => {
        //yawOffset = parseFloat(yawElem.textContent) || 0;
        //yawOffset = parseFloat(yawElem.textContent) || 0;
        // 初期姿勢を直立に設定
        console.log("pushed inityawBtn")
        
        //const uprightQuaternion = new THREE.Quaternion();
//        uprightQuaternion.setFromEuler(new THREE.Euler(0, THREE.Math.degToRad(90), 0, 'YXZ'));        
//        uprightQuaternion.setFromEuler(new THREE.Euler(0, THREE.Math.degToRad(90), 0, 'YXZ'));

//        uprightQuaternion.setFromEuler(new THREE.Euler(0, THREE.Math.degToRad(90), 0, 'XYZ'));
//        uprightQuaternion.setFromEuler(new THREE.Euler(0, THREE.Math.degToRad(90), 0, 'XYZ'));

          //uprightQuaternion.setFromEuler(new THREE.Euler(0, THREE.Math.degToRad(90), 0, 'XYZ'));
          //uprightQuaternion.setFromEuler(new THREE.Euler(0, 0, 0, 'XYZ'));


        //uprightQuaternion.setFromEuler(new THREE.Euler(0,0,  THREE.Math.degToRad(90), 'ZXY'));
        //uprightQuaternion.setFromEuler(new THREE.Euler(0, 0, 0, 'ZXY'));        
        //uprightQuaternion.setFromEuler(new THREE.Euler(0, 0, 0, 'YXZ'));        
        
        //base_q.copy(uprightQuaternion);

        
        base_q.copy(quaternion);
    });

    uploadSelectedFilesButton.addEventListener('click', uploadSelectedFiles);

    radioButtons.forEach(radio => {
        radio.addEventListener('change', () => {
            const button = document.querySelector('esp-web-install-button');
            button.manifest = `./manifest_${radio.value}.json`;
            button.classList.remove('invisible');
        });
    });

    document.getElementById('writeBasicInfoBtn').addEventListener('click', async () => {
        const mode = [];
        if (document.getElementById('bleHidUsbSerial').checked) mode.push('BLE HID & USB Serial');
        if (document.getElementById('usbHidBluetoothSerial').checked) mode.push('USB HID & Bluetooth Serial');
        if (document.getElementById('bleHidBluetoothSerial').checked) mode.push('BLE HID & Bluetooth Serial');
        const usbHidName = document.getElementById('usbHidName').value;
        const bleHidName = document.getElementById('bleHidName').value;
        const bluetoothSerialName = document.getElementById('bluetoothSerialName').value;

        const basicInfo = [
            'basicinfo',
            mode.join('|'),
            usbHidName,
            bleHidName,
            bluetoothSerialName
        ].join(',');

        if (port) {
            await writeToSerial(new TextEncoder().encode(basicInfo + '\n'));
            statusDiv.textContent += `基本情報書き込み: ${basicInfo}\n`;
        } else {
            statusDiv.textContent = "Web Serial API not supported or not connected.";
        }
    });

    window.onload = () => {
        fetchWebFiles();
        drawAxis(ctx);
    };

    const scene = new THREE.Scene();
    const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
    const renderer = new THREE.WebGLRenderer();
    renderer.setSize(400, 400);
    document.getElementById('threejsContainer').appendChild(renderer.domElement);

    const sphereGeometry = new THREE.SphereGeometry(1, 32, 32);
    const sphereMaterial = new THREE.MeshBasicMaterial({ color: 0xffffff, wireframe: true });
    const sphere = new THREE.Mesh(sphereGeometry, sphereMaterial);
    scene.add(sphere);

    //const stickGeometry = new THREE.BoxGeometry(0.24, 0.12, 0.48);
    const stickGeometry = new THREE.BoxGeometry(0.24, 0.12, 0.48);
    const orangeMaterial = new THREE.MeshBasicMaterial({ color: 0xffa500 });
    const blackMaterial = new THREE.MeshBasicMaterial({ color: 0x000000 });
    const whiteMaterial = new THREE.MeshBasicMaterial({ color: 0xffffff });

    
    const m5StickC = new THREE.Mesh(stickGeometry, [
        orangeMaterial,  // +X
        orangeMaterial,  // -X
        blackMaterial,  // +Y
        orangeMaterial,  // -Y
        orangeMaterial,   // +Z
        whiteMaterial   // -Z
    ]);
    
    const displayGeometry = new THREE.PlaneGeometry(0.2, 0.05);
    const displayMaterial = new THREE.MeshBasicMaterial({ color: 0x000000 });
    const display = new THREE.Mesh(displayGeometry, displayMaterial);
    display.position.set(0, 0.06, 0.24);  // スケールに合わせて位置を調整
    m5StickC.add(display);
    
    
    const markGeometry = new THREE.PlaneGeometry(0.04, 0.02);
    const markMaterial = new THREE.MeshBasicMaterial({ color: 0xffffff });
    const mark = new THREE.Mesh(markGeometry, markMaterial);
    mark.position.set(0, -0.06, 0.24);  // スケールに合わせて位置を調整
    m5StickC.add(mark);
    
    sphere.add(m5StickC);
    
    camera.position.z = -2;
    camera.lookAt(0, 0, 0);

    function animate() {
        requestAnimationFrame(animate);
        //m5StickC.setRotationFromQuaternion(quaternion);
        // `base_q` を基準としてオブジェクトを回転させる
        //m5StickC.setRotationFromQuaternion(base_q.clone().invert().multiply(quaternion));
        //m5StickC.setRotationFromQuaternion(base_q.clone().invert().multiply(quaternion));
        //m5StickC.setRotationFromQuaternion(base_q.clone().invert().multiply(quaternion));
        m5StickC.setRotationFromQuaternion(base_q.clone().invert().multiply(quaternion));



        renderer.render(scene, camera);
    }
    animate();

    function showPage(pageId) {
        document.querySelectorAll('.page').forEach(page => {
            page.style.display = 'none';
        });
        document.getElementById(pageId).style.display = 'block';
    }
});
