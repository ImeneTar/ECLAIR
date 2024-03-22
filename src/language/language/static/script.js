const startRecordButton = document.getElementById('startRecord');
const startSound = document.getElementById('startSound');
const stopSound = document.getElementById('stopSound');
const transcriptionParagraph = document.getElementById('transcription');

let mediaRecorder = null;
let audioChunks = [];

const defaultMicImageSrc = 'static/mic.png'; // Path to your default mic image
const activeMicImageSrc = 'static/mic_click.png';   // Path to your active (recording) mic image


function startRecording(stream) {
    mediaRecorder = new MediaRecorder(stream);
    audioChunks = [];

    mediaRecorder.addEventListener("dataavailable", event => {
        audioChunks.push(event.data);
    });

    mediaRecorder.addEventListener("stop", () => {
        const audioBlob = new Blob(audioChunks);
        sendAudioToServer(audioBlob);
    });

    mediaRecorder.start();
    startRecordButton.src = activeMicImageSrc;
    startSound.play();
}

// Function to stop recording
function stopRecording() {
    if (mediaRecorder && mediaRecorder.state !== "inactive") {
        mediaRecorder.stop();
        mediaRecorder.stream.getTracks().forEach(track => track.stop());
        startRecordButton.src = defaultMicImageSrc;
        stopSound.play();
    }
}


// Start recording when the button is pressed down
startRecordButton.addEventListener('mousedown', () => {
    transcriptionParagraph.textContent = "Recording... Speak now!";
    // Check if MediaRecorder is already instantiated and stop it
    if (mediaRecorder && mediaRecorder.state !== "inactive") {
        mediaRecorder.stop();
        mediaRecorder.stream.getTracks().forEach(track => track.stop());
    }
    console.log("Media reset")

    // Reset audioChunks for a new recording
    audioChunks = [];

    navigator.mediaDevices.getUserMedia({ audio: true })
        .then(startRecording)
        .catch(error => {
            console.error('Error getting audio stream:', error);
        });
});

// Stop recording when the button is released
startRecordButton.addEventListener('mouseup', stopRecording);


// Start recording when the button is pressed down
startRecordButton.addEventListener('touchstart', (e) => {
    e.preventDefault();
    transcriptionParagraph.textContent = "Recording... Speak now!";
    // Check if MediaRecorder is already instantiated and stop it
    if (mediaRecorder && mediaRecorder.state !== "inactive") {
        mediaRecorder.stop();
        mediaRecorder.stream.getTracks().forEach(track => track.stop());
    }
    console.log("Media reset")

    // Reset audioChunks for a new recording
    audioChunks = [];

    navigator.mediaDevices.getUserMedia({ audio: true })
        .then(startRecording)
        .catch(error => {
            console.error('Error getting audio stream:', error);
        });
});

// Stop recording when the button is released
startRecordButton.addEventListener('touchend', (e) => {
    e.preventDefault();
    stopRecording();
});




function sendAudioToServer(audioBlob) {
    const formData = new FormData();
    formData.append('audio', audioBlob);

    fetch('http://192.168.8.108:5000/transcribe', {
        method: 'POST',
        body: formData,
    })
    .then(response => response.json())
    .then(data => {
        transcriptionParagraph.textContent = data.transcription;
    })

    .catch(error => {
        console.error('Error transcribing audio:', error);
    });
}
