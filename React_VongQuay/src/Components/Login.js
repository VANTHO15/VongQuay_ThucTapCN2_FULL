import React, { useState, useEffect } from 'react';
import {
    useHistory,
  } from "react-router-dom";
import { Alert, AlertContainer } from "react-bs-notifier";
import firebaseConnect from '../firebaseConnect'
import { getDatabase, ref, onValue} from "firebase/database";

function Login(props) {
    let history = useHistory();
    function ChangeAccount()
    {   
        // console.log("Click 123");
        history.replace("/Change");
    }
    useEffect(()=>{
        async function getDataFromFirebase()
        {
            try
            {
                const db =  getDatabase();
                const starCountRef = ref(db, '/');
                onValue(starCountRef, (snapshot) => 
                {
                  var arrayData=[];
                  snapshot.forEach(element=>
                    {
                    const key = element.key;
                    const User = element.val().ID;
                    const Pass = element.val().PASS;
                    arrayData.push({
                      key:key,
                      User:User,
                      Pass:Pass
                     })
                    })
                //   console.log(arrayData);
                  setDataFirebase(arrayData);
                });
            } catch(error){
                console.log("Error", error.message);
            }
        }
        getDataFromFirebase();
    },[])
    const [UserName, setUserName]= useState("");
    const [PassWord, setPassWord]= useState("");
    const [isPass, setisPass]= useState(false);
    const [isHienThi, setisHienThi]= useState(false);
    const [DataFirebase, setDataFirebase]= useState(null);

    function handleChangeUser(e){
        const data = e.target.value;
        setUserName(data);
    }
    function handleChangePass(e){
        const data = e.target.value;
        setPassWord(data);
    }
    function handleLogin()
    {
        // console.log("Click");
        // console.log(UserName);
        // console.log(PassWord);
        DataFirebase.forEach((data)=>
        {
            if(UserName === data.User && PassWord === data.Pass)
            {
                localStorage.setItem("AccessToken", true);
                localStorage.setItem("Username",UserName);
                localStorage.setItem("key",data.key);
                setUserName("");
                setPassWord("");
                history.replace("/Main");
            }
        })
            setisHienThi(true);
            // console.log("Nhập Sai Tài Khoản Hoặc Mật Khẩu");
            setUserName("");
            setPassWord("");
    }
    function ShowPass()
    {
        if(!isPass)
        {
            return <img onClick={handleChanImage} className="anhoff" src="https://res.cloudinary.com/vantho15/image/upload/v1633546725/passOFF_z5xw2r.png" alt="MatOFF"/>
        }
        else
        return <img onClick={handleChanImage} className="anhoff" src="https://res.cloudinary.com/vantho15/image/upload/v1633546725/passON_x9limm.png" alt="MatOFF" />
    }
    function ShowInput()
    {
        if(!isPass)
        {
            return <input onChange={(e)=>handleChangePass(e)} value={PassWord} name="PassWord"  type="password" className="form-control input_pass" defaultValue="" placeholder="Password" />
        }
        else
        return <input onChange={(e)=>handleChangePass(e)} value={PassWord} name="PassWord"  type="text" className="form-control input_pass" defaultValue="" placeholder="Password" />
    }
    function handleChanImage()
    {
        setisPass(!isPass);
    }
    function handleOFFThongBao()
    {
        setisHienThi(false);
    }
    function HienThiThongBao()
    {
        if(isHienThi === true)
        {
            return (
                <AlertContainer>
                    <Alert type="danger" onDismiss={handleOFFThongBao} timeout={3000}>Wrong account or password, please try again !</Alert>
                </AlertContainer>
            )
        }
        return ;
    }
    return (
        <div className="GiuaManHinh">
       
          
         
            <div className="user_card">
                
                {HienThiThongBao()}

                <div className="d-flex justify-content-center">
                    <div className="brand_logo_container">
                    <img src="https://res.cloudinary.com/vantho15/image/upload/v1633134182/LoGo_s5cojv.png" className="brand_logo" alt="Logo" />                       
                    </div>
                </div>
                <div className="d-flex justify-content-center form_container">
                    <form >
                   
                    <div className="input-group mb-3">
                        <div className="input-group-append">
                        <span className="input-group-text"><i className="fa fa-user" /></span>
                        </div>
                        <input onChange={(e)=>handleChangeUser(e)} value={UserName} name="UserName" type="text" className="form-control input_user" defaultValue="" placeholder="UserName" />
                    </div>
                    <div className="input-group mb-2">
                        <div className="input-group-append">
                        <span className="input-group-text"><i className="fa fa-key" /></span>
                        </div>
                        {ShowInput()}
                        {ShowPass()}
                    </div>
                    <div className="ChangeAccount">
                        <div onClick={ChangeAccount}>Change Account</div>
                    </div>
                    <div className="d-flex justify-content-center mt-3 login_container">
                        <button type="reset" onClick={handleLogin} className="btn login_btn">Confirm</button>
                    </div>
                    </form>
                </div>
            </div>
           
           
       
       </div>
    );
}

export default Login;