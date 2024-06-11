import DeviceCard1 from '../../components/DeviceCard/Type1';
import './DoorLight.scss';
import DoorOpen from '../../assets/images/door-open.svg';
import DoorClose from '../../assets/images/door-close.svg';
import LightOn from '../../assets/images/light-on.svg';
import LightOff from '../../assets/images/light-off.svg';
import { get, ref } from 'firebase/database';
import { db } from '../../firebase';
import { useEffect, useState } from 'react';

function DoorLight(){
    const [doorInitValue, setDoorInitValue] = useState(false);
    const [lightInitValue, setLightInitValue] = useState(false);

    useEffect(() => {
        // Load the initial state from localStorage
        const savedDoorState = localStorage.getItem('doorState');
        const savedLightState = localStorage.getItem('lightState');

        if (savedDoorState !== null) {
            setDoorInitValue(JSON.parse(savedDoorState));
        } else {
            get(ref(db,'/door/door-1'))
                .then(snapshot => {
                    if (snapshot.exists()) {
                        const value = snapshot.val().state;
                        setDoorInitValue(value);
                        localStorage.setItem('doorState', JSON.stringify(value));
                    } else {
                        console.log("No data available");
                    }
                })
                .catch(err => console.log("Error:", err));
        }

        if (savedLightState !== null) {
            setLightInitValue(JSON.parse(savedLightState));
        } else {
            get(ref(db,'/light/light-1'))
                .then(snapshot => {
                    if (snapshot.exists()) {
                        const value = snapshot.val().state;
                        setLightInitValue(value);
                        localStorage.setItem('lightState', JSON.stringify(value));
                    } else {
                        console.log("No data available");
                    }
                })
                .catch(err => console.log("Error:", err));
        }
    }, []);
    
    return (
        <>
            <h3>Door & Light</h3>
            <div className='doorLight__list'>
                <DeviceCard1 device="Door" imgOn={DoorOpen} imgOff={DoorClose} initValue={doorInitValue} />
                <DeviceCard1 device="Light" imgOn={LightOn} imgOff={LightOff} initValue={lightInitValue} />
            </div>
        </>
    );
}

export default DoorLight;
