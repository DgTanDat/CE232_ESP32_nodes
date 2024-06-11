import { NavLink } from 'react-router-dom';
import LogoSmartHome from '../../assets/images/smartHome.png';
import './Sidebar.scss';

function Sidebar() {
    return (
        <>
            <div className='sidebar'>
                <div className='sidebar__title'>
                    <img src={LogoSmartHome} alt="Logo Smart Home" />
                    <h3>Smart Home</h3>
                </div>

                <div className='sidebar__list'>
                    <ul>
                        <NavLink to="/dashboard/overview">
                            <li>
                                <i class="fa-solid fa-gauge"></i>
                                <span>Overview</span>
                            </li>
                        </NavLink>

                        <NavLink to="/dashboard/door-light">
                            <li>
                                <i class="fa-solid fa-door-open"></i>
                                <span>Door & Light</span>
                            </li>
                        </NavLink>

                        <NavLink to="/dashboard/fan">
                            <li>
                                <i class="fa-solid fa-fan"></i>
                                <span>Fan</span>
                            </li>
                        </NavLink>

                        <NavLink to="/dashboard/sensor">
                            <li>
                                <i class="fa-solid fa-temperature-three-quarters" style={{textAlign: "center"}}></i>
                                <span>Temperature & Humidity </span>
                            </li>
                        </NavLink>
                    </ul>
                </div>

                <div className='sidebar__social'>
                    <h5>Follow us</h5>
                    <a href="https://www.facebook.com/Google"><i class="fa-brands fa-facebook fa-lg"></i></a>
                    <a href='https://www.instagram.com/google/'><i class="fa-brands fa-square-instagram fa-lg"></i></a>
                    <a href='https://twitter.com/Google'><i class="fa-brands fa-square-x-twitter fa-lg"></i></a>
                </div>
            </div>
        </>
    );
}

export default Sidebar;