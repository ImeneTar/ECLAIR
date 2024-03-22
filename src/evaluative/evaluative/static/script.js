const triggerButton = document.getElementById("correct");
triggerButton.addEventListener("click", function() {
    triggerButton.classList.add("button-clicked_correct");
    fetch('/send_feedback/1', { method: 'POST' })
        .catch(error => {
            console.error('Error:', error);
        });
    setTimeout(function() {
        triggerButton.classList.remove("button-clicked_correct");
    }, 200);
});

const triggerButton_wrong = document.getElementById("wrong");
triggerButton_wrong.addEventListener("click", function() {
    triggerButton_wrong.classList.add("button-clicked_wrong");
    fetch('/send_feedback/-1', { method: 'POST' })
        .catch(error => {
            console.error('Error:', error);
        });
    setTimeout(function() {
        triggerButton_wrong.classList.remove("button-clicked_wrong");
    }, 200);
});



