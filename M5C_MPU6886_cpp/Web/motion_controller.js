document.addEventListener('DOMContentLoaded', () => {
    const uploadSelectedFilesButton = document.getElementById('uploadSelectedFilesButton');
    const webFilesSelect = document.getElementById('webFilesSelect');
    const statusDiv = document.getElementById('status');
    const radioButtons = document.querySelectorAll('input[name="type"]');
    let port;
    let yawOffset = 0;
    let quaternion = new THREE.Quaternion();
    let lastPitch = 0, lastRoll = 0;
    const pitchElem = document.getElementById('pitch');
    const rollElem = document.getElementById('roll');
    const yawElem = document.getElementById('yaw');
    const quaternionElem = document.getElementById('quaternion');
    const pitchRollCanvas = document.getElementById('pitchRollCanvas');
    const ctx = pitchRollCanvas.getContext('2d');
    const bluePoints = [];
    const sphereIntersections = [];
    let buffer = '';

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
        try {
            while (true) {
                const { value, done } = await reader.read();
                if (done) {
                    reader.releaseLock();
                    break;
                }
                const text = new TextDecoder().decode(value);
                buffer += text;
                processBuffer();
            }
        } catch (error) {
            console.error(error);
        } finally {
            reader.releaseLock();
        }
    }

    function processBuffer() {
        let endIndex;
        while ((endIndex = buffer.indexOf('\r\n')) >= 0) {
            const line = buffer.slice(0, endIndex);
            buffer = buffer.slice(endIndex + 2);
            appendSerialOutput(line + '\n');
            handleSerialData(line);
        }
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
                let qx = parseFloat(parts[10]);
                let qy = parseFloat(parts[11]);
                let qz = parseFloat(parts[12]);
                let qw = parseFloat(parts[13]);

                if (document.getElementById('eulerToQuaternion').checked) {
                    const euler = new THREE.Euler(
                        THREE.Math.degToRad(pitch),
                        THREE.Math.degToRad(roll),
                        THREE.Math.degToRad(yaw),
                        'YXZ' // 回転順序を変更
                    );
                    quaternion.setFromEuler(euler);
                    qx = quaternion.x;
                    qy = quaternion.y;
                    qz = quaternion.z;
                    qw = quaternion.w;
                }

                pitchElem.textContent = pitch.toFixed(3);
                rollElem.textContent = roll.toFixed(3);
                yawElem.textContent = yaw.toFixed(3);
                quaternionElem.textContent = `${qx.toFixed(3)}, ${qy.toFixed(3)}, ${qz.toFixed(3)}, ${qw.toFixed(3)}`;

                quaternion.set(qx, qy, qz, qw);

                updatePitchRollCanvas(pitch, roll);

                // Update last values
                lastPitch = pitch;
                lastRoll = roll;
            }
        } else if (line.startsWith('selected_pk')) {
            // Plot previous pitch/roll as blue point
            plotPoint(lastPitch, lastRoll, 'blue');
            // Plot intersection with sphere
            plotIntersectionWithSphere(quaternion);
        }
    }

    function updatePitchRollCanvas(pitch, roll) {
        ctx.clearRect(0, 0, pitchRollCanvas.width, pitchRollCanvas.height);
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
        const direction = new THREE.Vector3(1, 0, 0); // 長い面の中心方向（x軸方向）
        direction.applyQuaternion(quaternion).normalize();
        const intersection = direction.clone().multiplyScalar(1); // 半径1の球との交点
        const color = intersection.z >= 0 ? 0x0000ff : 0x800080; // 表側：青、裏側：紫

        const geometry = new THREE.SphereGeometry(0.05, 32, 32);
        const material = new THREE.MeshBasicMaterial({ color: color });
        const sphere = new THREE.Mesh(geometry, material);

        sphere.position.copy(intersection);
        sphere.scale.setScalar(1 / camera.position.distanceTo(sphere.position)); // 距離によらず大きさを固定

        scene.add(sphere);
        sphereIntersections.push(sphere);
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
        yawOffset = parseFloat(yawElem.textContent) || 0;
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

    window.onload = fetchWebFiles;

    const scene = new THREE.Scene();
    const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
    const renderer = new THREE.WebGLRenderer();
    renderer.setSize(400, 400);
    document.getElementById('threejsContainer').appendChild(renderer.domElement);

    const sphereGeometry = new THREE.SphereGeometry(1, 32, 32);
    const sphereMaterial = new THREE.MeshBasicMaterial({ color: 0xffffff, wireframe: true });
    const sphere = new THREE.Mesh(sphereGeometry, sphereMaterial);
    scene.add(sphere);

    const stickGeometry = new THREE.BoxGeometry(0.6, 0.2, 0.15);
    const orangeMaterial = new THREE.MeshBasicMaterial({ color: 0xffa500 });
    const blackMaterial = new THREE.MeshBasicMaterial({ color: 0x000000 });

    const m5StickC = new THREE.Mesh(stickGeometry, [
        orangeMaterial,  // +X
        orangeMaterial,  // -X
        orangeMaterial,  // +Y
        orangeMaterial,  // -Y
        blackMaterial,   // +Z
        orangeMaterial   // -Z
    ]);

    const displayGeometry = new THREE.PlaneGeometry(0.5, 0.1);
    const displayMaterial = new THREE.MeshBasicMaterial({ color: 0x000000 });
    const display = new THREE.Mesh(displayGeometry, displayMaterial);
    display.position.set(0, 0.1, 0.075);
    m5StickC.add(display);

    const markGeometry = new THREE.PlaneGeometry(0.1, 0.05);
    const markMaterial = new THREE.MeshBasicMaterial({ color: 0xffffff });
    const mark = new THREE.Mesh(markGeometry, markMaterial);
    mark.position.set(0, -0.1, 0.075);
    m5StickC.add(mark);

    sphere.add(m5StickC);

    camera.position.z = 2;

    function animate() {
        requestAnimationFrame(animate);
        m5StickC.setRotationFromQuaternion(quaternion);
        renderer.render(scene, camera);
    }
    animate();

    function showPage(pageId) {
        document.querySelectorAll('.page').forEach(page => {
            page.style.display = 'none';
        });
        document.getElementById(pageId).style.display = 'block';
    }

    window.showPage = showPage; // Make showPage globally accessible
});