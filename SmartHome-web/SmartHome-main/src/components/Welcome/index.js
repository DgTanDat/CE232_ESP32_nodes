import { Link } from "react-router-dom";
import logo from "../../assets/images/smartHome.png";
import Clock from "../Clock";
import './Welcome.scss';

function Welcome() {
    return (
        <div className="container">
            <div style={{textAlign: "center"}}>Welcome to your Smart Home!!!</div>
            <Clock></Clock>
            <img src={logo} alt="logo"></img>  
            <Link to="/login">
                <button className="btn btn-success">Go to login page</button>
            </Link>
        </div>

    );
}

export default Welcome;