document.addEventListener('DOMContentLoaded', () => {
    const uploadSelectedFilesButton = document.getElementById('uploadSelectedFilesButton');
    const webFilesSelect = document.getElementById('webFilesSelect');
    const statusDiv = document.getElementById('status');
    const radioButtons = document.querySelectorAll('input[name="type"]');
    let port;
    let yawOffset = 0;
    let base_q = new THREE.Quaternion();
    let q_ref = new THREE.Quaternion();

    let quaternion = new THREE.Quaternion();
    let rotate_q = new THREE.Quaternion();

    let horizontal_q = new THREE.Quaternion();
    let upright_q = new THREE.Quaternion().setFromEuler(new THREE.Euler(1.57, 1.57, 0));

    let lastPitch = 0, lastRoll = 0;
    const pitchElem = document.getElementById('pitch');
    const rollElem = document.getElementById('roll');
    const yawElem = document.getElementById('yaw');
    const quaternionElem = document.getElementById('quaternion');
    const pitchRollCanvas = document.getElementById('pitchRollCanvas');
    const ctx = pitchRollCanvas.getContext('2d');
    const bluePoints = [];
    const sphereIntersections = [];

    const referencesData = [];

    const flushBtn = document.getElementById('flushBtn');
    const readBtn = document.getElementById('readBtn');
    const deleteBtn = document.getElementById('deleteBtn');
    const showBtn = document.getElementById('showBtn');
    const showReferencesBtn = document.getElementById('showReferencesBtn');
    const rebootBtn = document.getElementById('rebootBtn');
    const resetBtn = document.getElementById('resetBtn');
    const qoffsetBtn = document.getElementById('qoffsetBtn');
    const qinitializeBtn = document.getElementById('qinitializeBtn');

    let savedPort = null; // 以前選択したポートを保存する変数


    document.getElementById('connectBtn').addEventListener('click', async () => {
        await connectSerial();
    });


    async function connectSerial() {
        if (!("serial" in navigator)) {
            statusDiv.textContent = "Web Serial API not supported.";
            return null;
        }
    
        if (port && port.readable) {
            statusDiv.textContent = "Port is already open.";
            return port;
        }
    
        try {
            if (!savedPort) {
                port = await navigator.serial.requestPort();
                savedPort = port; // ポートを保存
            } else {
                port = savedPort; // 保存されたポートを使用
            }
            await port.open({ baudRate: 115200 });
            readLoop();
            setupEventListeners(); // Set up event listeners after the port is open
        } catch (error) {
            if (error.name === 'NotFoundError') {
                statusDiv.textContent = "No port selected by the user.";
            } else {
                statusDiv.textContent = `Error: ${error.message}`;
            }
        }
    }
    function setupEventListeners() {
        flushBtn.addEventListener('click', () => sendCommand('FLUSH'));
        readBtn.addEventListener('click', () => sendCommand('READ'));
        deleteBtn.addEventListener('click', () => sendCommand('DELETE'));
        showBtn.addEventListener('click', () => sendCommand('SHOW'));
        showReferencesBtn.addEventListener('click', () => sendCommand('SHOW_REFERENCES'));
        rebootBtn.addEventListener('click', () => sendCommand('REBOOT'));
        resetBtn.addEventListener('click', () => sendCommand('RESET'));
        qoffsetBtn.addEventListener('click', () => sendCommand('QOFFSET'));
        qinitializeBtn.addEventListener('click', () => sendCommand('QINIT'));


        function sendCommand(command) {
            const serialInput = document.getElementById('serialInput');
            serialInput.value = command;
            document.getElementById('sendBtn').click();
        }
    }

    async function readLoop() {
        const reader = port.readable.getReader();
        let readBuffer = '';
        try {
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
        } catch (error) {
            // Attempt to reconnect if a NetworkError occurs
            if (error.name === 'NetworkError' || error.name === 'DOMException') {
                console.error('The device has been lost:', error);
                statusDiv.textContent = "The device has been lost. Attempting to reconnect...";
                
                try {
                    await reconnectSerial();
                } catch (reconnectError) {
                    console.error('Reconnection failed:', reconnectError);
                    statusDiv.textContent = `Reconnection failed: ${reconnectError.message}`;
                }
            } else {
                console.error('An unexpected error occurred:', error);
            }
            if (error.name === 'NetworkError' || error.name === 'DOMException') {
                console.error('The device has been lost:', error);
                statusDiv.textContent = "The device has been lost. Please reconnect.";
                // Optionally, you can attempt to reconnect or handle the error as needed
            } else {
                console.error('An unexpected error occurred:', error);
            }
        } finally {
            reader.releaseLock();
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

    function appendSerialOutput(text) {
        const serialOutput = document.getElementById('serialOutput');
        serialOutput.textContent += text;
        serialOutput.scrollTop = serialOutput.scrollHeight;
    }

    function handleSerialData(line) {
        if (line.startsWith('sensor_data')) {
            const parts = line.split(',');
            if (parts.length === 14) {
                let pitch = parseFloat(parts[7]);
                let roll = parseFloat(parts[8]);
                let yaw = parseFloat(parts[9]);

                let qw = parseFloat(parts[10]);
                let qx = parseFloat(parts[11]);
                let qy = parseFloat(parts[12]);
                let qz = parseFloat(parts[13]);

                pitchElem.textContent = pitch.toFixed(3);
                rollElem.textContent = roll.toFixed(3);
                yawElem.textContent = yaw.toFixed(3);
                quaternionElem.textContent = `${qx.toFixed(3)}, ${qy.toFixed(3)}, ${qz.toFixed(3)}, ${qw.toFixed(3)}`;

                quaternion.set(-qx, qz, qy, qw); // 寝かせてオフセットするとただしい

                updatePitchRollCanvas(pitch, roll);


                lastPitch = pitch;
                lastRoll = roll;
            }
        } else if (line.startsWith('selected_pk')) {
            const parts = line.split(',');
            const roll = parseFloat(parts[1]);
            const pitch = parseFloat(parts[2]);
            plotPoint(pitch, roll, 'red');
            plotIntersectionWithSphere(quaternion, "selected_pk");
    
            // Find the closest reference
            let minAngle = Infinity;
            let closestReference = null;
            referencesData.forEach(reference => {
                const refQuaternion = new THREE.Quaternion(-reference.qx, reference.qz, reference.qy, reference.qw);
                const angle = quaternion.angleTo(refQuaternion);
                if (angle < minAngle) {
                    minAngle = angle;
                    closestReference = reference;
                }
            });
    
            if (closestReference) {
                // Output the closest reference
                console.log('Closest Reference:', closestReference);
    
                // Change the color of the closest reference point
                plotIntersectionWithSphere(new THREE.Quaternion(-1 * closestReference.qx, closestReference.qz, closestReference.qy, closestReference.qw), "closest_reference");

                setTimeout(() => {
                    if (selectedSphere) {
                        selectedSphere.material.color.set(0xff6347); // tomato color
                    }
                }, 500); // Change the delay as needed
            }
    
        } else if (line.startsWith('pk_references')) {
            const parts = line.split(',');
            const reference = {
                index: parseInt(parts[2]),
                roll: parseFloat(parts[8]),
                pitch: parseFloat(parts[9]),
                yaw: parseFloat(parts[10]),
                qw: parseFloat(parts[11]),
                qx: parseFloat(parts[12]),
                qy: parseFloat(parts[13]),
                qz: parseFloat(parts[14]),
                hid_input: parts[15],
                hid_inputs: parts.slice(16, 24).join(''),
                hid_input_acc_threshold: parseFloat(parts[25])
            };
            referencesData.push(reference);
            displayReferencesTable(referencesData);

            plotPoint(parseFloat(reference.pitch), parseFloat(reference.roll), 'orange');
            plotIntersectionWithSphere(new THREE.Quaternion(-1 * reference.qx, reference.qz, reference.qy, reference.qw),"pk_references");
        };
    }

    function updatePitchRollCanvas(pitch, roll) {
        ctx.clearRect(0, 0, pitchRollCanvas.width, pitchRollCanvas.height);
        drawAxis(ctx);
        /*
        ctx.beginPath();
        ctx.arc(pitchRollCanvas.width / 2, pitchRollCanvas.height / 2, 5, 0, 2 * Math.PI);
        ctx.fillStyle = 'red';
        ctx.fill();
        */
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

    function plotPoint(pitch, roll, color = 'blue') {
        const x = (roll + 180) * (pitchRollCanvas.width / 360);
        const y = (pitch + 90) * (pitchRollCanvas.height / 180);
        console.log(`Plotting point at (${x}, ${y}) with color ${color}`); // Debugging: Log the point coordinates
        bluePoints.push({ x, y, color });
        ctx.beginPath();
        ctx.arc(x, y, 5, 0, 2 * Math.PI);
        ctx.fillStyle = color;
        ctx.fill();
    }

    function plotReferencesPoint(pitch, roll) {
        plotPoint(pitch, roll, 'orange');
    }

    function plotIntersectionWithSphere(q, type) {
        // Ensure direction is along the z-axis
        const direction = new THREE.Vector3(0, 0, 1);
        rotate_q = base_q.clone().multiply(q_ref).invert().multiply(q);
        direction.applyQuaternion(rotate_q).normalize();
        console.log(rotate_q);
        
        const intersection = direction.clone().multiplyScalar(1);
        console.log(intersection.z);
        let color = 'red';
        if (type == "selected_pk") {
            color = intersection.z >= 0 ? 0x0000ff : 0x800080; //奥)blue or 手前)purple
        } else if (type == "pk_references") {
            color = intersection.z >= 0 ? 0xff6347 : 0xffa500; //tomato or orange
        } else if (type == "closest_reference") {
            color = 0x00ff00; // green for the closest reference
        }
    
        const distance = camera.position.distanceTo(intersection);
        const scaleFactor = Math.max(0.7, 0.5 * distance); // Ensure a minimum size
    
        const geometry = new THREE.SphereGeometry(0.05 * scaleFactor, 32, 32);
        const material = new THREE.MeshBasicMaterial({ color: color });
        const sphere = new THREE.Mesh(geometry, material);
    
        sphere.position.copy(intersection);
        scene.add(sphere);
        sphereIntersections.push(sphere);
    
        // Track the selected sphere
        if (type == "closest_reference") {
            selectedSphere = sphere;
        }

        // Debugging: Log the intersection position
        console.log('Intersection Position:', intersection);
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

    async function writeToSerial(data) {
        const writer = port.writable.getWriter();
        await writer.write(data);
        writer.releaseLock();
    }

    async function uploadSelectedFiles() {
        const selectedFiles = Array.from(webFilesSelect.selectedOptions).map(option => option.value);
        if (selectedFiles.length === 0) {
            statusDiv.textContent = "No files selected.";
            return;
        }

        for (const file of selectedFiles) {
            const response = await fetch(file);
            const data = await response.arrayBuffer();
            await writeToSerial(new Uint8Array(data));
            statusDiv.textContent += `Uploaded: ${file}\n`;
        }
    }

    document.getElementById('connectBtn').addEventListener('click', async () => {
        await connectSerial();
    });

    let isCalibrated = false;
    const calibrationStatusElem = document.createElement('span');
    calibrationStatusElem.textContent = 'not GUI calibrated';
    document.getElementById('initYawBtn').insertAdjacentElement('afterend', calibrationStatusElem);


    document.getElementById('sendBtn').addEventListener('click', async () => {
        const input = document.getElementById('serialInput').value;
        if (port && input) {
            await writeToSerial(new TextEncoder().encode(input + '\n'));
        }
    });

    document.getElementById('initYawBtn').addEventListener('click', () => {
        console.log("pushed initYawBtn");
        base_q.copy(quaternion);
        isCalibrated = true;
        calibrationStatusElem.textContent = '';
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



    function showPage(pageId) {
        document.querySelectorAll('.page').forEach(page => {
            page.style.display = 'none';
        });
        document.getElementById(pageId).style.display = 'block';
    }

    // グローバルスコープに関数を移動
    window.showPage = showPage;


    window.onload = () => {
        //fetchWebFiles();
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

    const baseUprightCheckbox = document.getElementById('baseUprightCheckbox');

    function animate() {
        requestAnimationFrame(animate);
        if (baseUprightCheckbox.checked) {
            q_ref = upright_q;
        } else {
            q_ref = horizontal_q;
        }
        rotate_q = base_q.clone().multiply(q_ref).invert().multiply(quaternion);
        m5StickC.setRotationFromQuaternion(rotate_q);
        renderer.render(scene, camera);
    }
    animate();


    function displayReferencesTable(data) {
        const table = document.createElement('table');
        table.classList.add('table', 'table-striped');

        const header = table.createTHead();
        const headerRow = header.insertRow();
        const headers = ['Index', 'Roll', 'Pitch', 'Yaw', 'Quaternion (qx, qy, qz, qw)', 'HID Input', 'HID Inputs', 'HID Input Acc Threshold'];
        headers.forEach(text => {
            const cell = document.createElement('th');
            cell.textContent = text;
            headerRow.appendChild(cell);
        });

        const tbody = table.createTBody();
        data.forEach(item => {
            const row = tbody.insertRow();
            row.insertCell().textContent = item.index;
            row.insertCell().textContent = item.roll;
            row.insertCell().textContent = item.pitch;
            row.insertCell().textContent = item.yaw;
            row.insertCell().textContent = `${item.qx}, ${item.qy}, ${item.qz}, ${item.qw}`;
            row.insertCell().textContent = item.hid_input;
            row.insertCell().textContent = item.hid_inputs;
            row.insertCell().textContent = item.hid_input_acc_threshold;
        });

        const container = document.getElementById('referencesTableContainer');
        container.innerHTML = '';
        container.appendChild(table);
    }
});