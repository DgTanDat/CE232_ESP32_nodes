import { ref, update } from "firebase/database";
import { db } from "../../firebase";


function RemoveTimerBtn() {

    const handleRemove = () => {
        update(ref(db,'light/light-1'), {
            isSetTime: false
        });
        alert("Remove successfully");
    }

    return (
        <button className="btn btn-danger" onClick={handleRemove}>Clear</button>

    );
}

export default RemoveTimerBtn;