let msgCounterTest = 0;

document.addEventListener('keydown', (e) => {
    if(e.key === "l") {
        const time = new Date(Date.now());
        addEntryToLog(`${time.getHours()}:${time.getMinutes()}:${time.getSeconds()}; lorem ipsum dolor sit amet ${++msgCounterTest}`);
    } else if (e.key === 's') {
        let sensorValuesToWrite = 50;
        const sensorWritePeriod = 500; //ms

        let left = 10.0;
        let right = 12.5;
        let ceiling = 8.2;
        let root = 2.4;

        const sensorWriteInterval = setInterval(() => {
            if (sensorValuesToWrite > 0) {
                displaySensorValues(left.toFixed(1), right.toFixed(1), ceiling.toFixed(1), root.toFixed(1));

                left -= 0.1;
                right -= 0.1;
                ceiling -= 0.1;
                root += 0.2;

                sensorValuesToWrite -= 1;
            } else {
                clearInterval(sensorWriteInterval);
            }
        }, sensorWritePeriod)
    } else if (e.key === 'p') {
        let currentDistance = 0.0;
        let targetDistance = 18.0;

        const sensorWritePeriod = 100; //ms

        const sensorWriteInterval = setInterval(() => {
            // console.log(currentDistance + " - " + targetDistance)
            if (currentDistance <= targetDistance + 0.05) {
                displaySensorValues(42, 42, 42, currentDistance.toFixed(1));

                currentDistance += 0.1;
            } else {
                clearInterval(sensorWriteInterval);
            }
        }, sensorWritePeriod)
    }
})