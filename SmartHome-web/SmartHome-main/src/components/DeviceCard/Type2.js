import { useEffect, useState } from "react";
import './Type2.scss';
import { onValue, ref, update } from "firebase/database";
import { db } from "../../firebase";


function DeviceCard2(props) {
    const { device, imgOn, imgOff, initValue} = props;
    const [levelFan, setLevelFan] = useState(0);

    // Set initial value and set color for initially active button
    useEffect(() => {
        console.log('useEffect 1');
        changeLevelBtn(initValue);
        setLevelFan(initValue);
    }, [initValue])

    // Update realtime the active button
    useEffect(() => {
        changeLevelBtn(levelFan);
    }, [levelFan])

    const handleChangeLevel = (e) => {
        var level = e.target.value;

        changeLevelBtn(level);

        // Write data to Firebase and localStorage
        localStorage.setItem('fanLevel', level);

        update(ref(db,'/fan/fan-1'), {
            level: level
        });

        setLevelFan(level);
    }   


    const changeLevelBtn = (level) => {
        const btnArr = document.querySelectorAll('.round-btn');
        for (let i = 0 ; i < btnArr.length; i++) {
            if (i == level) 
                btnArr[i].classList.add('active-btn');   
            else 
                btnArr[i].classList.remove('active-btn');    
        }
    }

    // Sync data between Firebase and localStorage
    useEffect(() => {
        const unsubcribe = onValue(ref(db,'/fan/fan-1/level'), (snapshot) => {
            var level = snapshot.val();
            setLevelFan(level);
            localStorage.setItem('fanLevel',level);
        })

        return () => unsubcribe();
    }, [])
    

    return (
        <>
            <div className="card2__container">
                <div className='card2__center'>
                    <img src={(levelFan > 0) ? imgOn : imgOff}></img>
                    <p>{device}</p>
                </div>

                <div className='card2__bottom'>
                    <button className="round-btn" value={0} onClick={handleChangeLevel}>0</button>
                    <button className="round-btn" value={1} onClick={handleChangeLevel}>1</button>
                    <button className="round-btn" value={2} onClick={handleChangeLevel}>2</button>
                    <button className="round-btn" value={3} onClick={handleChangeLevel}>3</button>
                </div>
            </div>
        </>
    );
}

export default DeviceCard2;

