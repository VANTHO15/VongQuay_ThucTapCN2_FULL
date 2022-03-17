import React, { useState, useEffect } from 'react';
import { useHistory } from 'react-router';
import { Alert, AlertContainer } from "react-bs-notifier";
import firebaseConnect from '../firebaseConnect'
import { getDatabase, ref, set , push, onValue} from "firebase/database";

function Account(props) {
    let history = useHistory();
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
    const [MacID, setMacID]= useState("");
    const [isShowMac, setisShowMac]= useState(false);
    const [iShowPass, setiShowPass]= useState(false);
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
    function handleChangeMacID(e){
        const data = e.target.value;
        setMacID(data);
    }
    function handleDangKi()
    {
        // console.log(UserName);
        // console.log(PassWord);
        // console.log(MacID);

        DataFirebase.forEach((data)=>
        {
            if(MacID === data.key)
            {
                localStorage.removeItem("AccessToken");
                const db = getDatabase();
                set(ref(db,MacID+"/"),{
                        ID:UserName,
                        PASS: PassWord
                })
                .then(()=>{
                    // alert("Thành Công");
                })
                .catch((er)=>{
                    setisHienThi(true);
                })

                setUserName("");
                setPassWord("");
                setMacID("");
                history.replace("/");
            }
            else
            {
                setUserName("");
                setPassWord("");
                setMacID("");
                setisHienThi(true);
            }
        })
    }
    function GoToLogin()
    {
        history.replace("/");
    }
    function ShowPassMAC()
    {
        if(!isShowMac)
        {
            return <img onClick={handleChanImageMAC} className="anhoff" src="https://res.cloudinary.com/vantho15/image/upload/v1633546725/passOFF_z5xw2r.png" alt="MatOFF"/>
        }
        else
        return <img onClick={handleChanImageMAC} className="anhoff" src="https://res.cloudinary.com/vantho15/image/upload/v1633546725/passON_x9limm.png" alt="MatOFF" />
    }
    function ShowInputMAC()
    {
        if(!isShowMac)
        {
            return <input onChange={(e)=>handleChangeMacID(e)} value={MacID} name="MacID" type="password" className="form-control input_user" defaultValue="" placeholder="Mac ID" />
        }
        else
        return <input onChange={(e)=>handleChangeMacID(e)} value={MacID} name="MacID" type="text" className="form-control input_user" defaultValue="" placeholder="Mac ID" />
    }
    function handleChanImageMAC()
    {
        setisShowMac(!isShowMac);
    }
    function ShowPass()
    {
        if(!iShowPass)
        {
            return <img onClick={handleChanImage} className="anhoff" src="https://res.cloudinary.com/vantho15/image/upload/v1633546725/passOFF_z5xw2r.png" alt="MatOFF" />
        }
        else
        return <img onClick={handleChanImage} className="anhoff" src="https://res.cloudinary.com/vantho15/image/upload/v1633546725/passON_x9limm.png" alt="MatOFF"/>
    }
    function ShowInput()
    {
        if(!iShowPass)
        {
            return <input onChange={(e)=>handleChangePass(e)} value={PassWord} name="PassWord"  type="password" className="form-control input_pass" defaultValue="" placeholder="Password" />
        }
        else
        return <input onChange={(e)=>handleChangePass(e)} value={PassWord} name="PassWord"  type="text" className="form-control input_pass" defaultValue="" placeholder="Password" />
    }
    function handleChanImage()
    {
        setiShowPass(!iShowPass);
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
                    <Alert type="danger" onDismiss={handleOFFThongBao} timeout={3000}>Wrong MacID, please try again !</Alert>
                </AlertContainer>
            )
        }
        return ;
    }
    return (
        // <div className="d-flex justify-content-center">
        <div className="container-fluid GiuaManHinh">
          <div className="row">
            <div className="col"></div>
            <div className="col">
            <div className="user_cardDangKI">
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
                        <span className="input-group-text"><i className="fa fa-users" /></span>
                        </div>
                        {ShowInputMAC()}
                        {ShowPassMAC()}
                    </div>
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
                        <div onClick={GoToLogin}>Go To Login</div>
                    </div>
                    <div className="d-flex justify-content-center mt-3 login_container">
                        <button type="reset" onClick={handleDangKi} className="btn login_btn1">Confirm</button>
                    </div>
                    </form>
                </div>
            </div>
                </div>
                <div className="col"></div>
            </div>
           </div>
       
    );
}

export default Account;