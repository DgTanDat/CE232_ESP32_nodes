import { onAuthStateChanged } from "firebase/auth";
import { createContext, useEffect, useState } from "react";
import { useNavigate } from "react-router-dom";
import { auth } from "../firebase";
import LoadingIcon from '../assets/images/loading.gif';

export const AuthContext = createContext();

function AuthProvider({ children }) {
  const navigate = useNavigate();
  const [authUser, setAuthUser] = useState({});
  const [isLoading, setIsLoading] = useState(true);

  useEffect(() => {
    const unsubscribed = onAuthStateChanged(auth, (user) => {
      setIsLoading(false);
      if (user) {
        const { displayName, email, uid, photoURL } = user;
        setAuthUser({ displayName, email, uid, photoURL });

      } else { 
        navigate('/');
      }
    });



    return () => unsubscribed();
  }, []);

  useEffect(() => {
    if (authUser.uid) {
      //console.log("User logged in:", authUser); 
      navigate('/dashboard'); 
    }
  }, [authUser]);
  
  return (
    <AuthContext.Provider value={{ authUser }}>
      {isLoading ? <img src={LoadingIcon} style={{margin: "25vh 40vw", width: "20%"}}></img> : children}
    </AuthContext.Provider>
  );
}

export default AuthProvider;
