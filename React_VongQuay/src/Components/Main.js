import React, {useEffect , useRef, useState} from 'react';
import { useHistory } from 'react-router';
// npm install react-responsive --save
import { useMediaQuery } from 'react-responsive'
import firebaseConnect from '../firebaseConnect'
import { getDatabase, ref, set , push, onValue} from "firebase/database";

var interval;
function Main(){
// begin mqtt
    let history = useHistory();
    function handleLogour()
    {
      localStorage.removeItem("AccessToken");
      localStorage.removeItem("Username");
      localStorage.removeItem("key");
      history.replace("/");
    }

// responsive  
const isDesktopOrLaptop = useMediaQuery({query: '(min-width: 1224px)'})
const isTabletOrMobile = useMediaQuery({ query: '(max-width: 1224px)' });

// begin vòng quay
const [NutNhan,setNutNhan]=useState("dung");
const [Tang,setTang]=useState(0);
const [Kq,setKq]=useState(0);

let audio = new Audio('https://res.cloudinary.com/vantho15/video/upload/v1646758244/tick_sbmiue.mp3');
let ketthuc = new Audio('https://res.cloudinary.com/vantho15/video/upload/v1646759971/ketthuc_dmcrer.wav');
function FunctionChuyen()
{
  history.replace("/Main1");
}
function HienThiMenu()
{
  if(isDesktopOrLaptop)
  { 
    return(
      <nav className="navbar navbar-expand-sm navbar-dark bg-danger">	
            <div className="navbar-brand"> <i onClick={FunctionChuyen} class="fa fa-user-circle cach-chyen"> Go To 2</i></div>
            <button className="navbar-toggler d-lg-none" type="button" data-toggle="collapse" data-target="#collapsibleNavId" aria-controls="collapsibleNavId" aria-expanded="false" aria-label="Toggle navigation" />
            <div className="collapse navbar-collapse" id="collapsibleNavId">
                <ul className="navbar-nav mt-2 mt-lg-0  ">   
                    <div data-toggle="modal"  data-target="#myModal" className="nav-link" > <i class="fa fa-sign-out homeicon"></i> </div>
                </ul>
            </div>
        </nav>
    )
  }
  else if(isTabletOrMobile)
  {
    return(
      <nav className="navbar navbar-expand-sm navbar-dark bg-danger">	
      <div className="navbar-brand"> <i onClick={FunctionChuyen} class="fa fa-user-circle cach-chyen"> Go To 2</i></div>
      <button className="navbar-toggler d-lg-none" type="button" data-toggle="collapse" data-target="#collapsibleNavId" aria-controls="collapsibleNavId" aria-expanded="false" aria-label="Toggle navigation" />
      <div className="collapse navbar-collapse" id="collapsibleNavId">
          <ul className="navbar-nav mt-2 mt-lg-0  ">   
              <div data-toggle="modal"  data-target="#myModal" className="nav-link" > <i class="fa fa-sign-out homeicon"></i> </div>
          </ul>
      </div>
  </nav>
    ) 
  }
}

function HienThiContent()
{
  if(isDesktopOrLaptop)
  { 
    return(
      <div>
        <div className='arrow'> </div>
              <ul style={styte}> 
                  <li>
                      <div className='text' 
                        spellCheck='false'>Một Ly</div>
                  </li>
                  <li>
                      <div className='text' 
                        spellCheck='false'>Uống Hộ</div>
                  </li>
                  <li>
                      <div className='text' 
                        spellCheck='false'>Đồng Khởi</div>
                  </li>
                  <li>
                      <div className='text' 
                        spellCheck='false'>Hai Ly</div> 
                  </li>
                  <li>
                      <div className='text' 
                        spellCheck='false'>Một Ly</div>
                  </li>
                  <li>
                      <div className='text' 
                        spellCheck='false'>Bên Cạnh</div>
                  </li>
                  <li>
                      <div className='text' 
                        spellCheck='false'>Một Ly</div>
                  </li>
                  <li>
                      <div className='text' 
                        spellCheck='false'>Đồng Khởi</div>
                  </li>
                  <li>
                      <div className='text' 
                        spellCheck='false'>Nửa Ly</div>
                  </li>
                  <li>
                      <div className='text' 
                        spellCheck='false'>Mời Ly</div>
                  </li>
                  <li>
                      <div className='text' 
                        spellCheck='false'>Một Ly</div>
                  </li>
                  <li>
                      <div className='text' 
                        spellCheck='false'>Qua Lượt</div>
                  </li>
                </ul>
            {HienThiNut()}
            {FunShowResult()}
            
      </div>
    )
  }
  else if(isTabletOrMobile)
  {
    return(
      <div>
        <div className='arrowMobie'> </div>
              <p className='text-center moiquay'>Mời bạn quay</p>
              <ul style={styteMobie}> 
                  <li>
                      <div className='textMobie' 
                        spellCheck='false'>Một Ly</div>
                  </li>
                  <li>
                      <div className='textMobie' 
                        spellCheck='false'>Uống Hộ</div>
                  </li>
                  <li>
                      <div className='textMobie' 
                        spellCheck='false'>Đồng Khởi</div>
                  </li>
                  <li>
                      <div className='textMobie' 
                        spellCheck='false'>Hai Ly</div> 
                  </li>
                  <li>
                      <div className='textMobie' 
                        spellCheck='false'>Một Ly</div>
                  </li>
                  <li>
                      <div className='textMobie' 
                        spellCheck='false'>Bên Cạnh</div>
                  </li>
                  <li>
                      <div className='textMobie' 
                        spellCheck='false'>Một Ly</div>
                  </li>
                  <li>
                      <div className='textMobie' 
                        spellCheck='false'>Đồng Khởi</div>
                  </li>
                  <li>
                      <div className='textMobie' 
                        spellCheck='false'>Nửa Ly</div>
                  </li>
                  <li>
                      <div className='textMobie' 
                        spellCheck='false'>Mời Ly</div>
                  </li>
                  <li>
                      <div className='textMobie' 
                        spellCheck='false'>Một Ly</div>
                  </li>
                  <li>
                      <div className='textMobie' 
                        spellCheck='false'>Qua Lượt</div>
                  </li>
                </ul>
            {HienThiNutMobie()}
            {FunShowResultMobie()}
            
      </div>
    ) 
  }
}
// end responsive 

// firebase



function handleClickDung()
  {
    clearInterval(interval);
    setNutNhan('tiep');
    ketthuc.pause();
    ketthuc.currentTime = 0;
    ketthuc.play(); 
   
    if(((Tang %360)/30) - Math.round(((Tang %360)/30)) > 0)
    {
      setKq( Math.round(((Tang %360)/30)) + 1);
    }
    else
    {
      setKq( Math.round(((Tang %360)/30)));
    }

     
    
  }
  function handleClickQuay()
  {
    const db = getDatabase();
    set(ref(db,"KQ"+"/"),
           -1
    )
    set(ref(db,"TRANG"+"/"),
           1
    )
    setNutNhan('quay');
     interval = setInterval(() => {
      setTang(Tang => Tang + 33);

      audio.pause();
      audio.currentTime = 0;
      audio.play();

    }, 60);
  }
  function handleClickTiep()
  {
    setTang(0);
    setNutNhan('dung');

    const db = getDatabase();
    set(ref(db,"KQ"+"/"),
           Kq
    )
    set(ref(db,"TRANG"+"/"),
           1
    )
    // const db = getDatabase();
    // set(ref(db,"Data"+"/"),{
    //         KQ:Kq,
    //         TRANG:"1"
    // })
    // .then(()=>{
    //     // alert("Thành Công");
    // })
    
  }
   function HienThiNut()
   {
      if(NutNhan === "quay")
      {
        return <button className='spin-button' onClick={handleClickDung}>Dừng</button>
      }
      if(NutNhan === "dung")
      {
        return <button className='spin-button' onClick={handleClickQuay}>Quay</button>
      }
      if(NutNhan === "tiep")
      {
        return <button className='spin-button' onClick={handleClickTiep}>Tiếp</button>
      }
   }
   function HienThiNutMobie()
   {
      if(NutNhan === "quay")
      {
        return <button className='spin-buttonMobie' onClick={handleClickDung}>Dừng</button>
      }
      if(NutNhan === "dung")
      {
        return <button className='spin-buttonMobie' onClick={handleClickQuay}>Quay</button>
      }
      if(NutNhan === "tiep")
      {
        return <button className='spin-buttonMobie' onClick={handleClickTiep}>Tiếp</button>
      }
   }
   function FunShowResult()
   {
     switch(Kq)
     {
      case 0:
        {
          return  <h2 className='kq'> Mời Quay</h2>
          break;
        }
       case 1:
         {
           return  <h2 className='kq'> Qua Lượt</h2>
           break;
         }
       case 2:
         {
           return  <h2 className='kq'> Một Ly</h2>
           break;
         }
       case 3:
         {
           return  <h2 className='kq'> Mời Ly</h2>
           break;
         }
       case 4:
         {
           return  <h2 className='kq'> Nửa Ly</h2>
           break;
         }
       case 5:
         {
           return  <h2 className='kq'> Đồng Khởi</h2>
           break;
         }
       case 6:
         {
           return  <h2 className='kq'> Một Ly</h2>
           break;
         }
       case 7:
         {
           return  <h2 className='kq'> Bên Cạnh</h2>
           break;
         }
       case 8:
         {
           return  <h2 className='kq'> Một Ly</h2>
           break;
         }
       case 9:
         {
           return  <h2 className='kq'> Hai Ly</h2>
           break;
         }
       case 10:
         {
           return  <h2 className='kq'> Đồng Khởi</h2>
           break;
         }
       case 11:
         {
           return  <h2 className='kq'> Uống Hộ</h2>
           break;
         }
       case 12:
         {
           return  <h2 className='kq'> Một Ly</h2>
           break;
         }
       
     }
    }
   function FunShowResultMobie()
   {
     switch(Kq)
     {
      case 0:
        {
          return  <h2 className='kqMobie'> Mời Quay</h2>
          break;
        }
       case 1:
         {
           return  <h2 className='kqMobie'> Qua Lượt</h2>
           break;
         }
       case 2:
         {
           return  <h2 className='kqMobie'> Một Ly</h2>
           break;
         }
       case 3:
         {
           return  <h2 className='kqMobie'> Mời Ly</h2>
           break;
         }
       case 4:
         {
           return  <h2 className='kqMobie'> Nửa Ly</h2>
           break;
         }
       case 5:
         {
           return  <h2 className='kqMobie'> Đồng Khởi</h2>
           break;
         }
       case 6:
         {
           return  <h2 className='kqMobie'> Một Ly</h2>
           break;
         }
       case 7:
         {
           return  <h2 className='kqMobie'> Bên Cạnh</h2>
           break;
         }
       case 8:
         {
           return  <h2 className='kqMobie'> Một Ly</h2>
           break;
         }
       case 9:
         {
           return  <h2 className='kqMobie'> Hai Ly</h2>
           break;
         }
       case 10:
         {
           return  <h2 className='kqMobie'> Đồng Khởi</h2>
           break;
         }
       case 11:
         {
           return  <h2 className='kqMobie'> Uống Hộ</h2>
           break;
         }
       case 12:
         {
           return  <h2 className='kqMobie'> Một Ly</h2>
           break;
         }
       
     }
   }
 
  // Math.floor(Math.random()*5000 + 2000));
  const styte ={
    width: '25em' ,
    height: '25em',
    border: '1px solid black',
    position: 'relative',
    padding: '0',
    margin: '1em auto',
    borderRadius: '50%',
    liststyle: 'none',
    overflow: 'hidden',
    transform: 'rotate(' + Tang + 'deg)',
    transition: 'ease-out',
    // transitionDuration: '0.7s'
  }
  const styteMobie ={
    width: '20em' ,
    height: '20em',
    border: '1px solid black',
    position: 'relative',
    padding: '0',
    margin: '1em auto',
    borderRadius: '50%',
    liststyle: 'none',
    overflow: 'hidden',
    transform: 'rotate(' + Tang + 'deg)',
    transition: 'ease-out',
    // transitionDuration: '0.7s'
  }

//  MAIN 
    return (
        <div>
                {/* The Modal */}
                <div className="modal" id="myModal">
                  <div className="modal-dialog">
                    <div className="modal-content">
                      {/* Modal Header */}
                      <div className="modal-header">
                        <h4 className="modal-title">🚫❌🔕</h4>
                        <button type="button" className="close" data-dismiss="modal">×</button>
                      </div>
                      {/* Modal body */}
                      <div className="modal-body">
                       Are you sure you want to sign out !😭☹️
                      </div>
                      {/* Modal footer */}
                      <div className="modal-footer">
                        <button onClick={handleLogour} type="button" data-dismiss="modal" className="btn btn-danger">OK 😥😥</button>
                        <button type="button" className="btn btn-primary" data-dismiss="modal">Close 🥰</button>
                      </div>
                    </div>
                  </div>
                </div>
  {/* begin menu   */}
        { HienThiMenu()}
  {/* ///////////////////// */}
       {HienThiContent()}
   </div>
    );
}

export default Main;