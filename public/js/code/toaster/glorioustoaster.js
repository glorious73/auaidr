function showToaster(message, typeColor) {
  // Get the toaster ID
  let toaster = document.getElementById("toaster");
  // Change the text of the poster
  toaster.innerHTML = message;
  // Change background color
  toaster.style.backgroundColor = typeColor;
  // Add the "show" class to the toaster
  toaster.className = "show";
  // After 3 seconds, hide it
  setTimeout(function(){ toaster.className = toaster.className.replace("show", ""); }, 3000);
}

function showSuccess(message) { showToaster(message, "#629632"); } //greenish

function showError(message) { showToaster(message, "#CC0000"); } //reddish

function showWarning(message) { showToaster(message, "#FFA500"); } //orange

function showInfo(message) { showToaster(message, "#6666FF"); } //blueish
