<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<title>ESP32 Speeduino Dashboard</title>
<style>
body { font-family: Arial; background: #111; color: #eee; }
h1 { text-align:center; }
.data { display:flex; justify-content: space-around; padding: 20px; }
.data div { background:#222; padding:10px 20px; border-radius:10px; }
</style>
</head>
<body>
<h1>Speeduino Dashboard</h1>
<div class="data">
  <div>RPM: <span id="rpm">0</span></div>
  <div>TPS: <span id="tps">0</span></div>
  <div>CLT: <span id="clt">0</span></div>
  <div>CEL: <span id="cel">0</span></div>
</div>

<script>
async function updateTelemetry(){
  try{
    let response = await fetch("/telemetry");
    let data = await response.json();
    document.getElementById("rpm").innerText = data.RPM;
    document.getElementById("tps").innerText = data.TPS;
    document.getElementById("clt").innerText = data.CLT;
    document.getElementById("cel").innerText = data.CEL;
  }catch(e){
    console.log("Error fetching telemetry:", e);
  }
}

setInterval(updateTelemetry, 200); // update co 200ms
</script>
</body>
</html>
