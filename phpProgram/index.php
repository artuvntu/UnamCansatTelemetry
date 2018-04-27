<html>
  <head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width">
  <meta name="description" content="EkOrbit Telemetry view">
  <meta name="author" content="EkOrbit">
  <title>Compis</title>
  <script>
    var puntosCO2Raw = [];
    var puntosPPMRaw = [];
    var puntosUVRaw = [];
    var puntosO3 = [];
    var puntosNO2 = [];
    var blimps = [];
    var puntosRaw = [];
    
    for (var i = 0; i < 5; i++) {
      puntosRaw.push([]); 
    }
      <?php
        $nombreArchivo = "blimp01.eko";

        while ($nombreArchivo[6] != '4') {
          $archivo = fopen($nombreArchivo, "rb");
          $tamArchivo = filesize($nombreArchivo);
          $posicion = ftell($archivo);
          while($posicion < $tamArchivo) {
            $tipo = unpack("i", fread($archivo, 4));
            $valor = unpack("f", fread($archivo, 4));
            if($tipo[1] < 5){
              echo "puntosRaw[",$tipo[1],"].push(",$valor[1],");";
            }else{
              echo "console.log(\"Error de tipo ",$tipo,"\");";
            }
            $posicion = ftell($archivo);
          }
          echo "blimps.push(puntosRaw);puntosRaw = [];for (var i = 0; i < 5; i++)puntosRaw.push([]);";
          $nombreArchivo[6] = $nombreArchivo[6] + 1;
          echo "console.log(\"", $nombreArchivo, "\");";
          // $archivo = fopen($nombreArchivo, "rb");
          // $tamArchivo = filesize("blimp01.eko");
        }
      ?>

    // for (var i = 0; i < blimps[0].length; i++) {
    //   console.log(blimps[0][i]);
    // }
  </script>
  <link rel="stylesheet" href="./css/style.css">
<script src="./javaScript/inicializarWeb.js">
</script>
  </head>
  <body>
    <header>
      <div class="container">
        <div id="branding">
          <img class="imagenLogo" src="./img/logoUnam.jpg" >
        </div>
        <nav>
          <ul>
            <li class="current"><a href="index.html">Home</a></li>
            <!-- <li><a href="tableTest.php">Table</a></li> -->
            <!-- <li><a href="./php/lecturaInformacion.php">Services</a></li> -->
          </ul>
        </nav>
      </div>
    </header>
    <section id="showcase">
      <div class="container">
        <h1>Compis</h1> 
        HOLA<br>
        <script src="https://canvasjs.com/assets/script/canvasjs.min.js"></script>
 <!--        <script async defer 
        src="https://maps.googleapis.com/maps/api/js?key=AIzaSyDqYQwm-I1PQmpDh0Dc7G5WF2lz_y0yOFg&callback=initMap">
        </script> -->
        <div id="contenedorTelemetry">
          <div id="tablaCO2" style="height: 100%; width: 100%;"></div>
        </div>
        <div id="contenedorTelemetry">
          <div id="tablaPPM" style="height: 100%; width: 100%;"></div>
        </div>
        <div id="contenedorTelemetry">
          <div id="tablaUV" style="height: 100%; width: 100%;"></div>
        </div>
        <div id="contenedorTelemetry">
          <div id="tablaO3" style="height: 100%; width: 100%;"></div>
        </div>
        <div id="contenedorTelemetry">
          <div id="tablaNO2" style="height: 100%; width: 100%;"></div>
        </div>
        <!-- <div id ="contenedorTelemetry">
          <video width="100%" height="100%" controls>
          <source src="./img/videoStream.m4v" type="video/mp4">
          Your browser does not support the video tag.
          </video>
        </div> -->
<!--         <div id="contenedorTelemetry">
          <div id="map" style="height: 100%; width: 100%;"></div>
        </div>
      </div> -->
    </section>
    <footer>
      <p>Facultad de Ingeniería</p>
    </footer>
  </body>
</html>
