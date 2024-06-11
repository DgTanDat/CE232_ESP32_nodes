import { useContext, useState } from 'react';
import './Header.scss';
import { AuthContext } from '../../Context/AuthProvider';
import { auth } from '../../firebase';



function Header() {
    const [isShowing, setIsShowing] = useState(false);
    const data = useContext(AuthContext);

    // Display avatar or not
    //console.log(data.authUser.photoURL);


    // Split username from email 
    var username = "";
    if (data.authUser.displayName) username = data.authUser.displayName;
    else {
        let temp = data.authUser.email;
        let index = temp.indexOf("@");
        username = temp.slice(0, index);
    }

    // Show Header List
    const handleShowHeaderList = () => {
        var headerUsername = document.querySelector('.header__list');
        if (!isShowing) {
            headerUsername.classList.add('header__list__show');
        }
        else {
            headerUsername.classList.remove('header__list__show');
        }
        setIsShowing(!isShowing);
    }

    // Handle Log out
    
    const handleLogout = () => {
        auth.signOut();
    }
      

    return (
        <>
            <div className="header">
                <div className='header__user'>
                    <div className='header__usericon'>
                        {data.authUser.photoURL ? <img src={data.authUser.photoURL} alt="Avatar"></img> : <i class="fa-regular fa-circle-user fa-xl"></i>}
                    </div>

                    <div className='header__username' onClick={handleShowHeaderList}>
                        <div><b>{username}</b></div>
                        <i class="fa-solid fa-chevron-down"></i>
                    </div>

                    <div className='header__list'>
                        <ul>
                            <li><i class="fa-regular fa-user"></i>Profile</li>
                            <li><i class="fa-solid fa-gear"></i>Settings</li>
                            <li onClick={handleLogout} onBlur={handleLogout}><i class="fa-solid fa-right-from-bracket"></i>Logout</li>
                        </ul>
                    </div>
                </div>
            </div>
        </>
    );
}

export default Header;