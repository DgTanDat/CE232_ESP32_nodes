import { useEffect, useState } from 'react';
import FanOff from '../../assets/images/fan-off.svg';
import FanOn from '../../assets/images/fan-on.svg';
import DeviceCard2 from "../../components/DeviceCard/Type2";
import './Fan.scss';
import { get, ref } from 'firebase/database';
import { db } from '../../firebase';

function Fan(){
    const [initFanLevel, setInitFanLevel] = useState(false);

    useEffect(() => {
        const savedFanLevel = localStorage.getItem('fanLevel');

        if (savedFanLevel !== null) {
            setInitFanLevel(savedFanLevel);
        }
        else {
            get(ref(db,'/fan/fan-1'))
                .then((snapshot) => {
                    setInitFanLevel(snapshot.val().level);
                    localStorage.setItem('fanLevel', snapshot.val().level);
                })

                .catch((err) => console.log(err));
        }
    },[])

    return (
        <>
            <h3>Fan</h3>
            <div className='device__list'>
                <DeviceCard2 device="Fan" imgOn={FanOn} imgOff={FanOff} initValue={initFanLevel}></DeviceCard2>
            </div>
        </>

    );
}

export default Fan;


