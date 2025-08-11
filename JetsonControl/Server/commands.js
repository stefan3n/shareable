function sendTurnLeft()
{
    var host = window.location.host;
    var pwm = document.getElementById("pwm").value;
    fetch("http://" + host + "/turnLeft/" + pwm, {method: "POST"});
}

function sendTurnRight()
{
    var host = window.location.host;
    var pwm = document.getElementById("pwm").value;
    fetch("http://" + host + "/turnRight/" + pwm, {method: "POST"});
}

function sendForward()
{
    var host = window.location.host;
    var pwm = document.getElementById("pwm").value;
    fetch("http://" + host + "/forward/" + pwm, {method: "POST"});
}

function sendBackward()
{
    var host = window.location.host;
    var pwm = document.getElementById("pwm").value;
    fetch("http://" + host + "/backward/" + pwm, {method: "POST"});
}

function sendBreak()
{
    var host = window.location.host;
    fetch("http://" + host + "/break", {method: "POST"});
}

function sendEmergency()
{
    var host = window.location.host;
    fetch("http://" + host + "/emergency", {method: "POST"});
}

function sendEndEmergency()
{
    var host = window.location.host;
    fetch("http://" + host + "/endEmergency", {method: "POST"});
}

function sendStart()
{
    var host = window.location.host;
    fetch("http://" + host + "/start", {method: "POST"});
}

function moveOne()
{
    var host = window.location.host;
    var potentiometerValue = document.getElementById("actuator1").value;
    fetch("http://" + host + "/moveOne/" + potentiometerValue, {method: "POST"});
}

function moveTwo()
{
    var host = window.location.host;
    var potentiometerValue = document.getElementById("actuator2").value;
    fetch("http://" + host + "/moveTwo/" + potentiometerValue, {method: "POST"});
}

function moveThree()
{
    var host = window.location.host;
    var potentiometerValue = document.getElementById("actuator3").value;
    fetch("http://" + host + "/moveThree/" + potentiometerValue, {method: "POST"});
}

function moveOneUp()
{
    var host = window.location.host;
    fetch("http://" + host + "/moveOneUp/", {method: "POST"});
}

function moveTwoUp()
{
    var host = window.location.host;
    fetch("http://" + host + "/moveTwoUp/", {method: "POST"});
}

function moveThreeUp()
{
    var host = window.location.host;
    fetch("http://" + host + "/moveThreeUp/", {method: "POST"});
}

function moveOneDown()
{
    var host = window.location.host;
    fetch("http://" + host + "/moveOneDown/", {method: "POST"});
}

function moveTwoDown()
{
    var host = window.location.host;
    fetch("http://" + host + "/moveTwoDown/", {method: "POST"});
}

function moveThreeDown()
{
    var host = window.location.host;
    fetch("http://" + host + "/moveThreeDown/", {method: "POST"});
}

function goxyz()
{
    var host = window.location.host;
    var x = document.getElementById("xpos").value;
    var y = document.getElementById("ypos").value;
    var z = document.getElementById("zpos").value;
    fetch("http://" + host + "/goxyz/" + x + "/" + y + "/" + z, {method: "POST"});
}

function initialPos()
{
    var host = window.location.host;
    fetch("http://" + host + "/initialPos", {method: "POST"});
}

function rotateLeft()
{
    var host = window.location.host;
    fetch("http://" + host + "/rotateLeft", {method: "POST"});
}

function rotateRight()
{
    var host = window.location.host;
    fetch("http://" + host + "/rotateRight", {method: "POST"});
}

function rotateArmLeft()
{
    var host = window.location.host;
    fetch("http://" + host + "/rotateArmLeft", {method: "POST"});
}

function rotateArmRight()
{
    var host = window.location.host;
    fetch("http://" + host + "/rotateArmRight", {method: "POST"});
}

function stopRobot()
{
    var host = window.location.host;
    fetch("http://" + host + "/stopRobot", {method: "POST"});
}

function exit()
{
    var host = window.location.host;
    fetch("http://" + host + "/exit", {method: "POST"});
}

function autonomousMode()
{
    var host = window.location.host;
    fetch("http://" + host + "/autonomous", {method: "POST"});
}

function manualMode()
{
    var host = window.location.host;
    fetch("http://" + host + "/manual", {method: "POST"});
}

function calLeft()
{
    var host = window.location.host;
    var steps = document.getElementById("steps").value;
    fetch("http://" + host + "/calLeft/" + steps, {method: "POST"});
}

function calRight()
{
    var host = window.location.host;
    var steps = document.getElementById("steps").value;
    fetch("http://" + host + "/calRight/" + steps, {method: "POST"});
}

function clawRotate()
{
    var host = window.location.host;
    var deg = document.getElementById("degrees").value;
    fetch("http://" + host + "/clawRotate/" + deg, {method: "POST"});
}

function clawOpen()
{
    var host = window.location.host;
    fetch("http://" + host + "/clawOpen", {method: "POST"});
}

function clawClose()
{
    var host = window.location.host;
    fetch("http://" + host + "/clawClose", {method: "POST"});
}
