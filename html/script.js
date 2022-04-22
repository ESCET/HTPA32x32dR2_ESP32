

async function downloadBlob(content, filename, contentType) {
  // Create a blob
  var blob = new Blob([content], { type: contentType });
  var url = URL.createObjectURL(blob);

  // Create a link to download it
  var pom = document.createElement('a');
  pom.href = url;
  pom.setAttribute('download', filename);
  pom.click();
}

  function changeCategory(category) {
 
  datacounter = 0
  category = category;
  sensor_data_list = [];
}
async function htpaCatpure() {
 
  this.extractData(category.value, datacounter)
  datacounter++
  // myInterval =  setInterval(function () {
    
  // }, 3000);
}
function htpaClear(){
 
  // clearInterval(myInterval);
  sensor_data_list = [];
  datacounter = 0
}
async function extractData(category, datacounter) {
  let file_length = 10;
  const result = await fetch("/api/get");
  const htpa_data = await result.json();
 
  sensor_array = Array.from(htpa_data.array.split(','), Number);
  sensor_data_list.push(Array.from(htpa_data.array.split(','), Number))
    const newArr = [];

    

while(sensor_array.length) newArr.push(sensor_array.splice(0,32));


var colorscaleValue = [
  [0, '#ffcc00'],
  [1, '#ff0000']
];
 
var data2 = [
  {
    z: newArr,
    type: 'heatmap',
    // colorscale: colorscaleValue,
       colorscale: 'Bluered',
    // colorscale: 'Hot',
    showscale: false

  }
];

 Plotly.react('heatmap', data2);



  if (datacounter != 0 && datacounter % file_length == 0) {
    downloadBlob(sensor_data_list.join("\n"), category + "-" + Date.now() + '.csv', 'text/csv;charset=utf-8;')
    sensor_data_list = [];
  }

  
 

 

}
