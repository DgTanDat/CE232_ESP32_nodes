import { onValue, ref, update } from 'firebase/database';
import { db } from '../../firebase';
import './Type1.scss';
import { useEffect, useState } from 'react';
import SetTimerBtn from '../SetTimerBtn';
import RemoveTimerBtn from '../RemoveTimerBtn';

function DeviceCard1(props) {
    const { device, imgOn, imgOff, initValue } = props;
    const [isOn, setIsOn] = useState(initValue);
    const [isTimer, setIsTimer] = useState(false);
    const [time, setTime] = useState();
    const [timeNow, setTimeNow] = useState("");

    // Initialize the value for isOn variable
    useEffect(() => {
        setIsOn(initValue);
    }, [initValue]);

    

    const handleOn = (e) => {
        const toggle = e.target;
        const newState = toggle.checked;
        setIsOn(newState);

        // Save state to localStorage
        if (device === "Door") {
            localStorage.setItem('doorState', JSON.stringify(newState));
        } 
        else if (device === "Light") {
            localStorage.setItem('lightState', JSON.stringify(newState));
        }

        // Write data to Firebase
        if (device === "Door") {
            update(ref(db, '/door/door-1'), { state: newState });
        } 
        else if (device === "Light") {
            update(ref(db, '/light/light-1'), { state: newState });
        }
    }

    useEffect(() => {
        // Sync the state with Firebase
        if (device === "Door") {
            const doorRef = ref(db, '/door/door-1/state');
            let listener = onValue(doorRef, (snapshot) => {
                const value = snapshot.val();
                setIsOn(value);
                localStorage.setItem('doorState', JSON.stringify(value));
            });
            return () => listener();


        } else if (device === "Light") {
            const lightRef = ref(db, '/light/light-1/state');
            let listener = onValue(lightRef, (snapshot) => {
                const value = snapshot.val();
                setIsOn(value);
                localStorage.setItem('lightState', JSON.stringify(value));
            });

            return () => listener();

        }
    }, [device]);

    // Update the variable time and isTimer
    useEffect(() => {
        let listener = onValue(ref(db,'light/light-1'), (snapshot) => {
            let timeSet = snapshot.val().timeTurnOn;
            let timer = snapshot.val().isSetTime;
            
            if ((timer != isTimer && timer != null) || (time != timeSet)) {
                setTime(timeSet);
                setIsTimer(timer);     
            }
        })

        return () => listener();
    }, [isTimer]);


    // Call API to know the current time
    useEffect(() => {
        const updateTimeNow = setInterval(() => {
            fetch('https://worldtimeapi.org/api/timezone/Asia/Ho_Chi_Minh')
                .then(res => res.json())
                .then(data => setTimeNow(data.datetime.substring(11,19)))
                .catch(err => console.log(err));

            
        }, 1000);

        return () => clearInterval(updateTimeNow);
    }, []);


    if ((time + ":00") == timeNow && isTimer) {
        update(ref(db,'light/light-1'), {state: true});
        //console.log("Alert");
    }

    // if ((time + ":00") == timeNow && isTimer) {
    //     update(ref(db,'light/light-1'), {state: true});
    //     //console.log("Alert");
    // }

    return (
        <div className="card__container">
            <div className='card__center'>
                <img src={isOn ? imgOn : imgOff} alt="Device" />
                <p>{device}</p>
            </div>
            <div className='card__bottom'>
                <div className='card__bottom--main'>
                    {device === "Door" ? (isOn ? <div>UNLOCKED</div> : <div>LOCKED</div>) 
                    : (isOn ? <div>ON</div> : <div>OFF</div>)}
                    <label className="switch">
                        <input type="checkbox" onChange={handleOn} checked={isOn} />
                        <span className="slider round"></span>
                    </label>
                </div>

                {device === "Light" && 
                (
                    <div className='card__bottom--noti'>
                        <div className='card__bottom--setTime'>
                            <div>Set timer</div>
                            <SetTimerBtn></SetTimerBtn>
                            <RemoveTimerBtn></RemoveTimerBtn>
                        </div>
                        <div>Turn on at: {
                            isTimer && time
                        }</div>
                    </div>
                )}
            </div>
        </div>
    );
}

export default DeviceCard1;
