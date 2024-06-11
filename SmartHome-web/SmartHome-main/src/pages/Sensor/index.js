import { Chart as ChartJS, CategoryScale, LineElement, PointElement, LinearScale, Title, defaults, Tooltip} from "chart.js";
import { onValue, ref } from "firebase/database";
import { db } from "../../firebase";
import { Line } from "react-chartjs-2";
import { useEffect, useState } from "react";


function Sensor() {
    // defaults.responsive = true;
    // defaults.plugins.title.display = true;
    // defaults.plugins.title.align = "center";
    // defaults.plugins.title.font.size = "15px";
    // defaults.plugins.title.color = "black";

    ChartJS.register(CategoryScale, LineElement, PointElement, LinearScale, Title, Tooltip);

    const [data, setData] = useState([]);

    useEffect(() => {
        const unsubcribed = onValue(ref(db, '/sensor/sensor-1'), (snapshot) => {
            // Convert Object into array
            var temp = Object.values(snapshot.val());

            // Get the last 15 elements 
            if (temp.length > 15){
                temp = temp.slice(temp.length - 15, temp.length);
                console.log(temp);
            }
                
            setData(temp);  
        });

        return () => unsubcribed();
    },[]);

    //console.log(data);

    return (
        <>
            <h3>Temperature & Humidity</h3>
            <div className="container-sensors" style={{display: "flex", justifyContent: "space-around", marginTop: "100px", marginLeft: "30px"}}>
                
                <div className="tempChart" style={{width: "40%"}}>
                    <Line
                        data={{
                            //labels: ["10:20", "10:21", "10:22", "10:23", "10:24", "10:25"],
                            labels: data.map((item) => {
                                var pos = item.timeStamp.search(" ");
                                return item.timeStamp.substring(0,pos);
                            }),
                            datasets: [
                                {
                                    label: "Temperature",
                                    //data: [20, 25, 30, 27, 23, 50],
                                    data: data.map((item) => item.tempValue),
                                    borderColor: 'rgba(255, 99, 132, 1)',
                                    pointStyle: "circle",
                                    pointRadius: 5,
                                    pointHoverRadius: 8
                                }
                            ]
                        }}
                        options={{
                            plugins: {
                                title: {
                                    font: {
                                        size: 15
                                    },
                                    
                                    display: true,
                                    align: "center",
                                    text: "Temperature"
                                }
                            },
                            scales: {
                                x: {
                                    title: {
                                        display: true,
                                        text: 'Time',
                                        color: "darkblue"
                                    }
                                },
                                y: {
                                    title: {
                                        display: true,
                                        text: 'Temperature (℃)',
                                        color: "darkblue"
                                    },
                                    suggestedMin: 0,
                                    suggestedMax: 50
                                }
                            }
                        }}
                    ></Line>
                </div>

                <div className="humidChart" style={{width: "40%"}}>
                    <Line
                            data={{
                                //labels: ["10:20", "10:21", "10:22", "10:23", "10:24", "10:25"],
                                labels: data.map((item) => {
                                    var pos = item.timeStamp.search(" ");
                                    return item.timeStamp.substring(0,pos);
                                }),
                                datasets: [
                                    {
                                        label: "Temperature",
                                        //data: [40, 75, 73, 80, 77, 75],
                                        data: data.map((item) => item.humidValue),
                                        borderColor: 'rgba(255, 99, 132, 1)',
                                        pointStyle: "circle",
                                        pointRadius: 5,
                                        pointHoverRadius: 8
                                    }
                                ]
                            }}
                            options={{
                                plugins: {
                                    title: {
                                        font: {
                                            size: 15
                                        },
                                        align: 'center',
                                        display: true,
                                        text: "Humidity"
                                    }
                                },
                                scales: {
                                    x : {
                                        title: {
                                            display: true,
                                            text: 'Time',
                                            color: "darkblue"
                                        }
                                    },
                                    y: {
                                        title: {
                                            display: true,
                                            text: 'Humidity',
                                            color: "darkblue"
                                        },
                                        suggestedMin: 0,
                                        suggestedMax: 100
                                    }
                                }
                            }}
                        ></Line>
                </div>
            </div>
        </>
    );
}

export default Sensor;
