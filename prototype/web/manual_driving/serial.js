const p_tag = document.getElementById('p_tag');

let port;

let reader;

let outputStream;

const log = document.getElementById('log');

const input = document.getElementById('input');

document.getElementById('connect').addEventListener('click', async () => {

    if ('serial' in navigator) {
        try {
            // Request a port and open a connection
            port = await navigator.serial.requestPort();

            await port.open({ baudRate: 115200 });

            logMessage("Connected to Arduino");

            // Start reading data
            reader = port.readable.getReader();
            outputStream = port.writable.getWriter();
        } catch (error) {
            alert("Couldn't connect to the arduino. Try selecting a different device. Error: " + error.message);
            logMessage("Failed to connect: " + error.message);
        }
    } else {
        alert("Web Serial API not supported in this browser. Try a different browser. Text Uros if it doesn't work");
    }
});

// async function readData() {
//     while (port && reader) {
//         try {
//             const { value, done } = await reader.read();
//             if (done) break;
//             logMessage("Received: " + new TextDecoder().decode(value));
//         } catch (error) {
//             logMessage("Error reading data: " + error.message);
//             break;
//         }
//     }
// }

function logMessage(message) {
    console.log(message);
}

async function sendMessage(toSend, ID) {
    if (port && outputStream) {
        const data = document.getElementById(ID).value;

        await outputStream.write(new TextEncoder().encode(toSend + data + ';'));

        logMessage(`Sent: ${data}`);
    } else {
        alert("Arduino not connected. Connect the arduino.")
    }
}

/**
 * Sends string to Arduino via serial
 * @param {*} message  messago to send
 */
async function sendSerialToArduino(message) {
    if (port && outputStream) {
        await outputStream.write(new TextEncoder().encode(message));
        console.log(message);
    } else {
        alert("Arduino not connected. Connect the arduino.")
    }
}
