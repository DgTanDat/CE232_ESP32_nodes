import './DefaultHome.scss';
import { Outlet } from "react-router-dom";

function DefaultHome() {
    return (
        <>
            <div className="background">
                <Outlet></Outlet>
            </div>
        </>
    );
}

export default DefaultHome;