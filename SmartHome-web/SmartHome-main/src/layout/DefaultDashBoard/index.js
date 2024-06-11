import { Outlet } from "react-router-dom";
import Sidebar from "../../components/Sidebar";
import Header from "../../components/Header";
import Clock from "../../components/Clock";
import './DefaultDashBoard.scss';
import { useContext } from "react";
import { AuthContext } from "../../Context/AuthProvider";
import { auth } from "../../firebase";

function DefaultDashBoard() {
    // const dataUser = useContext(AuthContext);
    // console.log(dataUser);

    return (
        <div className="container-fluid">
                <Sidebar></Sidebar>
                <div>
                    <Header></Header>
                    <div className="noti">
                        <h4>Welcome to your smart home!</h4>
                        <Clock></Clock>
                        <Outlet></Outlet>
                    </div>
                </div>
        </div>
    );
}

export default DefaultDashBoard;