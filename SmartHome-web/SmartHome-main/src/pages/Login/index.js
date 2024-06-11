import bootstrap from '../../../node_modules/bootstrap/dist/js/bootstrap.min.js'; 

import './Login.scss';
import image from '../../assets/images/login.png';
import Logo from '../../assets/images/smartHome.png';
import { useState } from 'react';
import { signInWithEmailAndPassword, signInWithPopup } from 'firebase/auth';
import { auth, provider } from '../../firebase';
import { useNavigate } from 'react-router-dom';

function Login() {
    const [email, setEmail] = useState('');
    const [password, setPassword] = useState('');

    const navigate = useNavigate();

    const handleChangeEmail = (e) => setEmail(e.target.value);
    const handleChangePass = (e) => setPassword(e.target.value);

    const handleSubmit = (e) => {
        e.preventDefault();

        signInWithEmailAndPassword(auth, email, password)
            .then((userCredential) => {
                alert(`Welcome ${userCredential.user.email}`);

                // const toastLiveExample = document.getElementById('liveToast');
                // const toast = new bootstrap.Toast(toastLiveExample);
                // toast.show();
            })

            .catch(err => alert('Your email and passwords do not match'));
    }

    const handleLoginGG = (e) => {
        e.preventDefault();

        signInWithPopup(auth, provider)
            .then( userCredential => {
                alert(`Welcome ${userCredential.user.displayName}`);
            })

            .catch(err => console.log(err));
    }


    return (
        <>
            <div className='container' style={{ paddingTop: "5%" }}>
                <div className='sub-container'>
                    <div className='image'>
                        <img src={image} alt="Login image"></img>
                    </div>

                    <div className="signIn">
                        <h2>Sign In</h2>
                        <form action="cuahang.html" id="signInForm" onSubmit={handleSubmit}>
                            <div className="data">
                                <input type="email" placeholder="Email" id="email" value={email} onChange={handleChangeEmail} /><br />
                                <input type="password" placeholder="Password" id="password" value={password} onChange={handleChangePass} />
                            </div>

                            <div className="forgotPass"><a href="#">Forgot Password?</a></div>
                            <button className='btn btn-success' type="submit">Sign In</button>

                            <div className='signin__google'>
                                <div>Or sign in with</div>
                                <button className='btn btn-primary' onClick={handleLoginGG}>
                                    <i class="fa-brands fa-google" ></i>
                                    Sign in with Google
                                </button>
                            </div>

                            <div className="register">Don't have an account? <a href="#"><b>Register</b></a></div>
                        </form>
                    </div>
                </div>

                {/* <div className="toast-container position-fixed bottom-0 end-0 p-3">
                    <div id="liveToast" className="toast" role="alert" aria-live="assertive" aria-atomic="true">
                        <div className="toast-header">
                            <img src={Logo} className="rounded me-2" alt="..."/>
                                <strong className="me-auto">Smart Home</strong>
                                <small>Recently</small>
                                <button type="button" className="btn-close" data-bs-dismiss="toast" aria-label="Close"></button>
                        </div>
                        <div className="toast-body">
                            Sign in successfully
                        </div>
                    </div>
                </div> */}
            </div>
        </>
    );
}

export default Login;