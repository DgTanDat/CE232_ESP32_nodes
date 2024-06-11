import { ref, update } from 'firebase/database';
import { useState } from 'react';
import Button from 'react-bootstrap/Button';
import Modal from 'react-bootstrap/Modal';
import { db } from '../../firebase';

function SetTimerBtn() {
  const [show, setShow] = useState(false);
  //const [timer, setTimer] = useState(null);

  const handleClose = () => setShow(false);
  const handleShow = () => setShow(true);
  const handleSave = () => {
      var time = document.getElementById('timer').value;
      
      if (time) {
        update(ref(db, 'light/light-1'), {
          isSetTime: true,
          timeTurnOn: time
        });

        alert("Set timer successfully");
      }
      else {
        alert("Please enter your time");
        
      }


      handleClose();
        
  }

  return (
    <>
      <Button variant="success" onClick={handleShow}>
        Set
      </Button>

      <Modal show={show} onHide={handleClose}>
        <Modal.Header closeButton>
          <Modal.Title><i class="fa-regular fa-lightbulb" style={{color: "#FFD43B", marginRight: "15px"}}></i>Set the timer light</Modal.Title>
        </Modal.Header>

        <Modal.Body>
            <label htmlFor="timer" style={{marginRight: "20px", marginLeft: "35px"}}><b>Time to turn on: </b></label>
            <input type='time' id='timer'></input>
        </Modal.Body>
        <Modal.Footer>
          <Button variant="secondary" onClick={handleClose}>
            Close
          </Button>
          <Button variant="primary" onClick={handleSave}>
            Save
          </Button>
        </Modal.Footer>
      </Modal>
    </>
  );
}

export default SetTimerBtn;