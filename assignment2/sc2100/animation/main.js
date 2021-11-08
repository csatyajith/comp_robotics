const pointsURI = 'data.json';
fetch(pointsURI, {
    method: 'GET',
}).then(response => {
    return response.json();
}).then(data => {
    animate(data);
});

const animate = function(data) {
    var chart = new CanvasJS.Chart("chartContainer", {
        animationEnabled: true,
        animationDuration: 10000,
        height: 600,
        width: 1200,
        theme: "light2",
        title: {
            text: "Robot Movement",
            fontSize: 24,
        },
        data: [{
            type: "line",
            indexLabelFontSize: 16,
            dataPoints: data
        }]
    });
    chart.render();
}