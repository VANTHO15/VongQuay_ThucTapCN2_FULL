import  { initializeApp }  from "firebase/app";
const firebaseConfig = {
    apiKey: "AIzaSyBYrB0bsmk7Z_1yeWkVEP_tNlm8XtFJVww",
    authDomain: "nhiptim-fee3b.firebaseapp.com",
    databaseURL: "https://nhiptim-fee3b-default-rtdb.firebaseio.com",
    projectId: "nhiptim-fee3b",
    storageBucket: "nhiptim-fee3b.appspot.com",
    messagingSenderId: "583618501207",
    appId: "1:583618501207:web:99e14cc0f05dc2b7ab6112",
    measurementId: "G-BFD57Q1XZX"
};
const firebaseConnect  = initializeApp(firebaseConfig);
export default firebaseConnect;  