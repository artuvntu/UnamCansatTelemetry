function initMap() {
  var la = 19.328025;
  var lo = -99.181082;
  var ciudadUniversitaria = {lat: la,lng: lo };
  var map = new google.maps.Map(document.getElementById('map'), {
      zoom: 9,
      center: ciudadUniversitaria
  });

  var circulo = new google.maps.Circle({
            strokeColor: '#FF0000',
            strokeOpacity: 0.8,
            strokeWeight: 2,
            fillColor: '#FF0000',
            fillOpacity: 0.35,
            map: map,
            center: ciudadUniversitaria,
            radius: 8000
  });
  var la = 19.384532;
  var lo = -99.036675;
  var ciudadUniversitaria = {lat: la,lng: lo };
    var circulo = new google.maps.Circle({
            strokeColor: '#00FF00',
            strokeOpacity: 0.8,
            strokeWeight: 2,
            fillColor: '#00FF00',
            fillOpacity: 0.35,
            map: map,
            center: ciudadUniversitaria,
            radius: 8000
  });
    
  var la = 19.484210;
  var lo = -99.140899;
  var ciudadUniversitaria = {lat: la,lng: lo };
    var circulo = new google.maps.Circle({
            strokeColor: '#0000FF',
            strokeOpacity: 0.8,
            strokeWeight: 2,
            fillColor: '#0000FF',
            fillOpacity: 0.35,
            map: map,
            center: ciudadUniversitaria,
            radius: 8000
  });
}
window.onload = function () {
  var puntosTabla = [];
  var lineas = [];
  var tablas = [];
  // Entra a los blimps

  for (var i = 0; i < blimps.length; i++) {
  // Entra a los tipos de telemetrias
  console.log(i);
    puntosTabla.push([]);
    for (var j = 0; j < blimps[i].length; j++) {
      puntosTabla[i].push([]);
      puntosTabla[i][j].push({
        x:0,
        y:blimps[i][j][0],
      })
      if (j == lineas.length){
        lineas.push([]);
      }
      lineas[j].push({
        type: "line",
        dataPoints: puntosTabla[i][j]
      })
    }
  }
  tablas.push(new CanvasJS.Chart("tablaCO2",{
    title:{
      text: "Acelerometro"
    },
    axisY: {
      includeZero: false
    },
    data: lineas[0],
  }));
  tablas.push(new CanvasJS.Chart("tablaPPM",{
    title:{
      text: "Giroscopio"
    },
    axisY: {
      includeZero: false
    },
    data: lineas[1],
  }));
  tablas.push(new CanvasJS.Chart("tablaUV",{
    title:{
      text: "Bateria"
    },
    axisY: {
      includeZero: false
    },
    data: lineas[2],
  }));
  tablas.push(new CanvasJS.Chart("tablaO3",{
    title:{
      text: "Presion"
    },
    axisY: {
      includeZero: false
    },
    data: lineas[3],
  }));
  tablas.push(new CanvasJS.Chart("tablaNO2",{
    title:{
      text: "Temperatura"
    },
    axisY: {
      includeZero: false
    },
    data: lineas[4],
  }));
  
  var actualizar = function(){
    for (var i = 0; i < blimps.length; i++) {
      for (var j = 0; j < blimps[i].length; j++) {
        var xnueva = puntosTabla[i][j].length
        puntosTabla[i][j].push({
          x:xnueva,
          y:blimps[i][j][xnueva%blimps[i][j].length],
        })
        // if (puntosTabla[i][j].length>5){
        //   puntosTabla[i][j].shift();
        // }
      }
    }
    for (t of tablas){
      t.render();
    }
    // chart.render();
  }
      setInterval(function(){actualizar()}, 100);

  // var tablaMagnetometro = new CanvasJS.Chart("contenedorTablaMagnetometro",{
  //   backgroundColor: "#F5DEB3",
  //   title :{
  //     text: "Magnetometer",
  //   },
  //   axisY: {
  //     includeZero : false
  //   },
  //   data: [{
  //     type: "line",
  //     dataPoints: puntosMagnetometroX
  //   },{
  //     type: "line",
  //     dataPoints: puntosMagnetometroY
  //   },{
  //     type: "line",
  //     dataPoints: puntosMagnetometroZ
  //   }]
  // });
  // var valorX = 0;
  // var valorY = 100;
  // var tiempoIntervalo = 100;
  // var cantidadDatos = 20;
  // var actualizarTabla = function () {
  //   valorY = valorY + Math.round(5 + Math.random() * (-10));
  //   puntosGiroscopioX.push({
  //     x: valorX,
  //     y: valorY
  //   });
  //   valorY = valorY + Math.round(5 + Math.random() * (-10));
  //   puntosGiroscopioY.push({
  //     x: valorX,
  //     y: valorY
  //   });
  //   valorY = valorY + Math.round(5 + Math.random() * (-10));
  //   puntosGiroscopioZ.push({
  //     x: valorX,
  //     y: valorY
  //   });

  //   valorY = valorY + Math.round(5 + Math.random() * (-10));
  //   puntosMagnetometroX.push({
  //     x: valorX,
  //     y: valorY
  //   });
  //   valorY = valorY + Math.round(5 + Math.random() * (-10));
  //   puntosMagnetometroY.push({
  //     x: valorX,
  //     y: valorY
  //   });
  //   valorY = valorY + Math.round(5 + Math.random() * (-10));
  //   puntosMagnetometroZ.push({
  //     x: valorX,
  //     y: valorY
  //   });
  //   valorX++;
  //   if (puntosGiroscopioX.length > cantidadDatos){
  //     puntosGiroscopioX.shift();
  //     puntosGiroscopioY.shift();
  //     puntosGiroscopioZ.shift();

  //     puntosMagnetometroX.shift();
  //     puntosMagnetometroY.shift();
  //     puntosMagnetometroZ.shift();
  //   }
  //   tablaGyro.render();
  //   tablaMagnetometro.render();
  // };
  // actualizarTabla(cantidadDatos);
  // setInterval(function(){actualizarTabla(cantidadDatos)}, tiempoIntervalo);
  }