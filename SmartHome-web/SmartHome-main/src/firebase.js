// Import the functions you need from the SDKs you need
import { initializeApp } from "firebase/app";
import { getAuth } from "firebase/auth";
import { GoogleAuthProvider } from "firebase/auth";
import { getDatabase } from "firebase/database";
// TODO: Add SDKs for Firebase products that you want to use
// https://firebase.google.com/docs/web/setup#available-libraries

// Your web app's Firebase configuration
const firebaseConfig = {
  apiKey: "AIzaSyB-c6XS__rjVLDvjpKrvaiSbVp-_ZrZh38",
  authDomain: "awesomeapp-d8758.firebaseapp.com",
  databaseURL: "https://awesomeapp-d8758-default-rtdb.firebaseio.com",
  projectId: "awesomeapp-d8758",
  storageBucket: "awesomeapp-d8758.appspot.com",
  messagingSenderId: "35211960422",
  appId: "1:35211960422:web:0b768b844405b9e8ae8da7"
};

// Initialize Firebase
const app = initializeApp(firebaseConfig);

// Firebase authentication with Google 
export const provider = new GoogleAuthProvider();

// Initialize Firebase Authentication and get a reference to the service
export const auth = getAuth(app);

// Get a Realtime database reference
export const db = getDatabase();