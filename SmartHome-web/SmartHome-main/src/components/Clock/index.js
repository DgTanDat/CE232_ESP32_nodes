import React, { useState, useEffect } from 'react';

function Clock() {
  const [time, setTime] = useState('2024-06-07T00:00:00');

  // useEffect(() => {
  //   const timer = setInterval(() => {
  //     setTime(new Date());
  //   }, 1000);

  //   return () => {
  //     clearInterval(timer);
  //   };
  // }, []);

  useEffect(() => {
    const clock = setInterval(() => {
        fetch('https://worldtimeapi.org/api/timezone/Asia/Ho_Chi_Minh')
            .then(res => res.json())
            .then(data => setTime(data.datetime.substring(0,19)))
            .catch(err => console.log(err));
    }, 1000);

    return () => clearInterval(clock);
}, []);

  // Formatting for both date and time
  const date = time.substring(0,10);
  const timeString = time.substring(11,19); 

  return (
    <div>
      <h2 style={{ textAlign: 'center' }}>{date}, {timeString}</h2>
    </div>
  );
}

export default Clock;
