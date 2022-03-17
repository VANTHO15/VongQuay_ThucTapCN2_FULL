import React, {useEffect , useRef, useState} from 'react';
import { useHistory } from 'react-router';
// npm install react-responsive --save
import { useMediaQuery } from 'react-responsive'
import '../App.css';
import firebaseConnect from '../firebaseConnect'
import { getDatabase, ref, set , push, onValue} from "firebase/database";

var MangCauHoi = [
  {
    CauHoi:"1 + 1 = ?",
    KQ:"2",
    A:"1",
    B:"2",
    C:"3",
    D:"4"
  },
  {
    CauHoi:"Có 1 đàn chim đậu trên cành, người thợ săn bắn cái rằm. Hỏi chết mấy con?",
    KQ:"15 con",
    A:"1 con",
    B:"15 con",
    C:"Không biết",
    D:"0 con"
  },
  {
    CauHoi:"'Thính' được làm từ gì?",
    KQ:"Gạo",
    A:"Mía",
    B:"Thịt",
    C:"Gạo",
    D:"Củ cải"
  },
  {
    CauHoi:"Thành nhà Hồ được UNESO công nhận là di sản văn hoá thế giới nằm ở tỉnh nào?",
    KQ:"Thanh Hoá",
    A:"Ninh Bình",
    B:"Thanh Hoá",
    C:"Nghệ An",
    D:"Hà Tĩnh"
  },
  {
    CauHoi:"Con cua tám cẳng hai càng, bớt đi hai cẳng hỏi bò mấy chân ?",
    KQ:"4 chân",
    A:"1 chân",
    B:"2 chân",
    C:"6 chân",
    D:"4 chân"
  },
  {
    CauHoi:"Đâu là tên một loại chợ ?",
    KQ:"Cóc",
    A:"Ếch",
    B:"Nhái",
    C:"Thằn lằn",
    D:"Cóc"
  },
  {
    CauHoi:"Tượng đài Chiến thắng Điện Biên Phủ được dựng trên ngọn đồi nào?",
    KQ:"D1",
    A:"D1",
    B:"C1",
    C:"E1",
    D:"A1"
  },
  {
    CauHoi:"Đâu không phải là một tác phẩm của họa sĩ Trần Văn Cẩn ?",
    KQ:"Đôi bạn",
    A:"Mẹ tôi",
    B:"Em Thúy",
    C:"Đôi bạn",
    D:"Em gái tôi"
  },
  {
    CauHoi:"Nhạc sĩ nào là người sáng tác ca khúc 'Cây đàn sinh viên'?",
    KQ:"Quốc An",
    A:"Bảo Chấn",
    B:"Trần Tiến",
    C:"Trịnh Công Sơn",
    D:"Quốc An"
  },
  {
    CauHoi:"Bộ phim 'Chị Dậu' được chuyển thể từ tác phẩm văn học nào?",
    KQ:"Tắt đèn",
    A:"Người mẹ cầm súng",
    B:"Tắt đèn",
    C:"Vợ chồng A Phủ",
    D:"Tuổi thơ dữ dội"
  },
  {
    CauHoi:"Bảy chú lùn trong truyện cổ tích làm nghề gì ?",
    KQ:"Thợ săn",
    A:"Thợ rèn",
    B:"Thợ mỏ",
    C:"Thợ săn",
    D:"Thợ may"
  },
  {
    CauHoi:"Chiếc tàu ngầm xuất hiện đầu tiên trên thế giới là của nước nào?",
    KQ:"Mỹ",
    A:"Anh",
    B:"Mỹ",
    C:"Đức",
    D:"Pháp"
  },
  {
    CauHoi:"Bảy chú lùn trong truyện cổ tích làm nghề gì ?",
    KQ:"Z",
    A:"Z",
    B:"N",
    C:"Q",
    D:"R"
  },
  {
    CauHoi:"Gang là hợp kim của ?",
    KQ:"Sắt và Các bon",
    A:"Sắt và Silic",
    B:"Sắt và Lưu huỳnh",
    C:"Sắt và Các bon",
    D:"Sắt và Phốtpho"
  },
  {
    CauHoi:"1 hectogam bằng bao nhiêu gam?",
    KQ:"100",
    A:"1000",
    B:"100",
    C:"10",
    D:"10000"
  },
  {
    CauHoi:"Sân bay lớn nhất phía Bắc Việt Nam?",
    KQ:"Nội Bài",
    A:"Quảng Ninh",
    B:"Nội Bài",
    C:"Thợ săn",
    D:"Gia Lâm"
  },
  {
    CauHoi:"'Đường vô xứ Huế quanh quanh - Non xanh nước biếc như tranh ....' gì?",
    KQ:"họa đồ",
    A:"sơn dầu",
    B:"sơn mài",
    C:"kí họa",
    D:"họa đồ"
  },
  {
    CauHoi:"Dòng sông nào thuộc tỉnh Quảng Bình đã từng chứng kiến sự chia cắt đất nước?",
    KQ:"Sông Gianh",
    A:"Sông Nhật Lệ",
    B:"Sông Bến Hải",
    C:"Sông Hương",
    D:"Sông Gianh"
  },
  {
    CauHoi:"Nước nào sau đây không phải là một thành viên thường trực của hội đồng bảoan LHQ?",
    KQ:"Đức",
    A:"Pháp",
    B:"Anh",
    C:"Nga",
    D:"Đức"
  },
  {
    CauHoi:"Kênh đào Panama là kênh đào nối hai đại dương nào?",
    KQ:"Thái Bình Dương, Đại Tây Dương",
    A:"Thái Bình Dương, Đại Tây Dương",
    B:"Thái Bình Dương, Ấn Độ Dương",
    C:"Đại Tây Dương, Ấn Độ Dương",
    D:"Đại Tây Dương, Bắc Băng Dương"
  },
  {
    CauHoi:"Liên đoàn bóng đá chât Á được viết tắt như thế nào?",
    KQ:"AFC",
    A:"UEFA",
    B:"CAF",
    C:"CONMEBOL",
    D:"AFC"
  },
  {
    CauHoi:"SVĐ Anfield là sân nhà của đội bóng nào?",
    KQ:"Liverpool",
    A:"Everton",
    B:"Liverpool",
    C:"Arsenal",
    D:"Chelsea"
  },
  {
    CauHoi:"1 sào bắc bộ bằng ?",
    KQ:"360 m2",
    A:"100 m2",
    B:"1000 m2",
    C:"360 m2",
    D:"3600 m2"
  },
  {
    CauHoi:"Từ 'Thủy Hử' trong tác phẩm cùng tên có nghĩa là gì?",
    KQ:"Bến nước",
    A:"Bến nước",
    B:"Quán rượu",
    C:"Quán nước",
    D:"Anh hùng ca"
  },
  {
    CauHoi:"Phố cổ Hội An là một đô thị cổ nằm bên bờ sông nào?",
    KQ:"Thu Bồn",
    A:"Trà Khúc",
    B:"Sông Nhật Lệ",
    C:"Sông Hoài",
    D:"Thu Bồn"
  },
  {
    CauHoi:"Công nghệ hiện đại Bluetooth được đặt theo tên vị vua nước nào?",
    KQ:"Đan Mạch",
    A:"Tây Ban Nha",
    B:"Latvia",
    C:"Đan Mạch",
    D:"Bồ Đào Nha"
  },
  {
    CauHoi:"Địa danh Lam Sơn của cuộc khởi nghĩa Lam Sơn nay thuộc tỉnh nào?",
    KQ:"Thanh Hóa",
    A:"Nghệ An",
    B:"Thanh Hóa",
    C:"Hà Tĩnh",
    D:"Ninh Bình"
  },
  {
    CauHoi:"RAM là :",
    KQ:"Bộ nhớ truy cập ngẫu nhiên",
    A:"Bộ nhớ đọc và truy cập",
    B:"Bộ nhớ chỉ cho phép đọc",
    C:"Bộ nhớ truy cập ngẫu nhiên",
    D:"Bộ nhớ có thể ghi được"
  },
  {
    CauHoi:"Nhóm máu nào chỉ có thể truyền cho chính nó ?",
    KQ:"AB",
    A:"A",
    B:"B",
    C:"O",
    D:"AB"
  },
  {
    CauHoi:"Thúy Kiều mang họ gì?",
    KQ:"Vương",
    A:"Dương",
    B:"Trương",
    C:"Khương",
    D:"Vương"
  },
  {
    CauHoi:"SVĐ Anfield là sân nhà của đội bóng nào?",
    KQ:"Liverpool",
    A:"Everton",
    B:"Liverpool",
    C:"Arsenal",
    D:"Chelsea"
  },
  {
    CauHoi:"Đơn vị cường độ ḍòng điện Ampere được đặt theo tên nhà bác học người nướcnào?",
    KQ:"Pháp",
    A:"Pháp",
    B:"Anh",
    C:"Đức",
    D:"Italia"
  },
  {
    CauHoi:"Nước nào ở khu vực Đông Nam Á không có địa giới với bất kỳ nước khác?",
    KQ:"Philippines",
    A:"Malaysia",
    B:"Lào",
    C:"Philippines",
    D:"Thái Lan"
  },
  {
    CauHoi:"Việt Nam lần đầu tiên vô địch AFF Cup là vào năm nào?",
    KQ:"2008",
    A:"2004",
    B:"2008",
    C:"2008",
    D:"2010"
  },
  {
    CauHoi:"Truyện 'Vợ chồng A Phủ' trong tập truyện Tây Bắc của nhà văn Tô Hoài viết về người dân tộc nào?",
    KQ:"Vân Kiều",
    A:"Thái",
    B:"Vân Kiều",
    C:"Thái",
    D:"Tày"
  },
  {
    CauHoi:"Bà chúa thơ Nôm Hồ Xuân Hương có bài thơ ví thân phận người phụ nữ với loại trái cây nào?",
    KQ:"Trái mít",
    A:"Trái bưởi",
    B:"Trái cam",
    C:"Trái mít",
    D:"Trái thị"
  },
  {
    CauHoi:"Cầu thủ nào có biệt danh là 'Tóc đuôi ngựa thần thánh'?",
    KQ:"Roberto Baggio",
    A:"Luigi Di Biagio",
    B:"Roberto Baggio",
    C:"Salvatore Schillaci",
    D:"Lionel Messei"
  },
  {
    CauHoi:"Trái đất cách Mặt trời khoảng bao nhiêu triệu km?",
    KQ:"Khoảng 150 triệu km",
    A:"Khoảng 150 triệu km",
    B:"Khoảng 250 triệu km",
    C:"Khoảng 350 triệu km",
    D:"Khoảng 450 triệu km"
  },
  {
    CauHoi:"Các quốc gia trong khu vực Đông Nam Á có vua bao gồm Campuchia, Brunei,Thái Lan và …",
    KQ:"Malaysia",
    A:"Malaysia",
    B:"Indonesia",
    C:"Philippines",
    D:"Singapore"
  },
  {
    CauHoi:"Nhà thơ nào được tôn xưng là 'Nữ sĩ sông Thương' ?",
    KQ:"Anh Thơ",
    A:"Hằng Phương",
    B:"Ngân Giang",
    C:"Anh Thơ",
    D:"Xuân Quỳnh"
  },
  {
    CauHoi:"Vụ án Lệ Chi Viên nổi tiếng trong lịch sử Việt Nam gắn liền với quan đại thầnnào?",
    KQ:"Nguyễn Trãi",
    A:"Nguyễn Trãi",
    B:"Trần Nguyên Hãn",
    C:"Nguyễn Xí",
    D:"Nguyễn Du"
  },
  {
    CauHoi:"Loại axit nào là thành phần chủ yếu của nước cốt chanh?",
    KQ:"Axit xitric",
    A:"Axit oxalic",
    B:"Axit xitric",
    C:"Axit lactric",
    D:"Axit lactic"
  },
  {
    CauHoi:"Nước nào ở khu vực Nam Mỹ có diện tích lớn nhất?",
    KQ:"Brazil",
    A:"Achentina",
    B:"Venezuala",
    C:"Brazil",
    D:"Brazil"
  },
  {
    CauHoi:"Người mắc bệnh gì cần ăn nhạt?",
    KQ:"Tăng huyết áp",
    A:"Cận thị",
    B:"Tăng huyết áp",
    C:"Gù cột sống",
    D:"Viêm da cơ địa"
  },
  {
    CauHoi:"Dân tộc nào sau đây ở Việt Nam còn giữ chế độ mẫu hệ?",
    KQ:"Chăm",
    A:"Chăm",
    B:"Bana",
    C:"Vân Kiều",
    D:"Kinh"
  },
  {
    CauHoi:"Kênh truyền hình VTV3 được thành lập vào thời gian nào?",
    KQ:"31/3/1996",
    A:"31/3/1996",
    B:"31/3/1994",
    C:"31/3/1998",
    D:"31/3/2000"
  },
  {
    CauHoi:"Trong thần thoại Hy Lạp, Helios là vị thần nào?",
    KQ:"Thần mặt trời",
    A:"Thần mặt trăng",
    B:"Thần mặt trời",
    C:"Thần biển cả",
    D:"Thần trí tuệ"
  },
  {
    CauHoi:"Phong trào Cần Vương được vị vua nhà Nguyễn nào phát động?",
    KQ:"Hàm Nghi",
    A:"Hàm Nghi",
    B:"Thành Thái",
    C:"Hiệp Hòa",
    D:"Đồng Khánh"
  },
  {
    CauHoi:"Chùa Cầu ở Hội An còn có tên gọi khác là gì?",
    KQ:"Chùa Nhật Bản",
    A:"Chùa Trung Quốc",
    B:"Chùa Hàn Quốc",
    C:"Chùa Nhật Bản",
    D:"Chùa Triều Tiên"
  },
  {
    CauHoi:"Làng tranh dân gian Đông Hồ có con sông nào chảy qua?",
    KQ:"Sông Đuống",
    A:"Sông Đuống",
    B:"Sông Luộc",
    C:"Sông Cầu",
    D:"Sông Lục Nam"
  },
  {
    CauHoi:"Súng thần công và cổ lâu thuyền những phát minh của ai ở nước ta thời xưa?",
    KQ:"Hồ Nguyên Trừng",
    A:"Lê Quý Đôn",
    B:"Trần Nhật Duật",
    C:"Hồ Nguyên Trừng",
    D:"Phạm Ngũ Lão"
  },
  {
    CauHoi:"Đỉnh núi nào cao thứ hai Việt Nam ?",
    KQ:"Pu Ling Sung",
    A:"Fansipan",
    B:"Pu Ta Leng",
    C:"Bạch Mộc Lương Tử",
    D:"Pu Ling Sung"
  },
  {
    CauHoi:"Cùng với cây phong ba, cây bão táp, cây nào là biểu tượng của Trường Sa ?",
    KQ:"Bàng vuông",
    A:"Bàng vuông",
    B:"Bàng tròn",
    C:"Bàng chữ nhậ",
    D:"Bàng bầu dục"
  },
  {
    CauHoi:"'Con chim hồng tước nhỏ' là biệt danh của cầu thủ nào ?",
    KQ:"Garrincha",
    A:"Pele",
    B:"Vava",
    C:"Garrincha",
    D:"Ferrenc Pukas"
  },
  {
    CauHoi:"Hành tinh nào xa nhất trong Hệ Mặt Trời ?",
    KQ:"Sao Hải Vương",
    A:"Sao Hải Vương",
    B:"Sao Diêm Vương",
    C:"Sao Hải Vương",
    D:"(Sao Thổ"
  },
  {
    CauHoi:"Người được nhân dân đặt cái biệt hiệu 'Chúa Chổm' là :",
    KQ:"Lê Trang Tông",
    A:"Lê Huệ Tông",
    B:"Đồng Khánh",
    C:"Lê Trang Tông",
    D:"Trần Phế Đế"
  },
  {
    CauHoi:"Vạn Thắng Vương là ai ?",
    KQ:"Đinh Bộ Lĩnh",
    A:"Lê Hoàn",
    B:"Đinh Bộ Lĩnh",
    C:"Lê Lợi",
    D:"Nguyễn Huệ"
  },
  {
    CauHoi:"Địa danh 'Cần Giuộc' trong bài 'Văn Tế nghĩa Sĩ Cần Giuộc' hiện nay nằm ở tỉnh nào ?",
    KQ:"Long An",
    A:"Tiền Giang",
    B:"An Giang",
    C:"Long An",
    D:"Cần Thơ"
  },
  {
    CauHoi:"Ai được tôn xưng là Trần Thái Tổ ?",
    KQ:"Trần Cảnh",
    A:"Trần Cảnh",
    B:"Trần Liễu",
    C:"Trần Thừa",
    D:"Trần Quốc Tuấn"
  },
  {
    CauHoi:"Dạ dày của động vật nhai lại có mấy ngăn?",
    KQ:"4 ngăn",
    A:"1 ngăn",
    B:"2 ngăn",
    C:"3 ngăn",
    D:"4 ngăn"
  },
  {
    CauHoi:"Chỉ số nào đánh giá khả năng nhận thức, hiểu và truyền đạt cảmxúc ?",
    KQ:"EQ",
    A:"IQ",
    B:"EQ",
    C:"AQ",
    D:"MQ"
  },
]
function Main(){
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
const [ThuTu,setThuTu]=useState(0);
const [ShowA,setShowA]=useState(true);
const [ShowB,setShowB]=useState(true);
const [ShowC,setShowC]=useState(true);
const [ShowD,setShowD]=useState(true);
const [ShowKQ,setShowKQ]=useState(false);

const [SaiA,setSaiA]=useState(false);
const [SaiB,setSaiB]=useState(false);
const [SaiC,setSaiC]=useState(false);
const [SaiD,setSaiD]=useState(false);

function FunctionChuyen()
{
  history.replace("/Main");
}
function HienThiMenu()
{
  if(isDesktopOrLaptop)
  { 
    return(
      <nav className="navbar navbar-expand-sm navbar-dark bg-danger">	
            <div className="navbar-brand"> <i  onClick={FunctionChuyen}  class="fa fa-user-circle cach-chyen"> Go To 1</i></div>
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
            <div className="navbar-brand"> <i  onClick={FunctionChuyen}  class="fa fa-user-circle cach-chyen"> Go To 1</i></div>
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
let BatDauChoi = new Audio('https://millionaire-school.netlify.app/sounds/lets_play.mp3');
let ChoDoi = new Audio('https://millionaire-school.netlify.app/sounds/easy.mp3');
let Sai = new Audio('https://millionaire-school.netlify.app/sounds/wrong_answer.mp3');
let Dung = new Audio('https://millionaire-school.netlify.app/sounds/correct_answer.mp3');
// câu hỏi

function FuncCauTiepTheo()
{
  setShowKQ(false);
  setThuTu( Math.round(Math.random() *MangCauHoi.length))  // const rand = min + Math.random() * (max - min);
  setShowA(true);
  setShowB(true);
  setShowC(true);
  setShowD(true);
  BatDauChoi.currentTime = 0;
  BatDauChoi.play();  

  setSaiA(false);
  setSaiB(false);
  setSaiC(false);
  setSaiD(false);

  // ChoDoi.currentTime = 0;
  // ChoDoi.play();  
  const db = getDatabase();
    set(ref(db,"KQ"+"/"),
           -1
    )
    set(ref(db,"TRANG"+"/"),
           2
    )

} 
function Func5050()
{
  if(MangCauHoi[ThuTu].A !== MangCauHoi[ThuTu].KQ)
  {
    setShowA(false);
  }
  else
  if(MangCauHoi[ThuTu].B !== MangCauHoi[ThuTu].KQ)
  {
    setShowB(false);
  }
  if(MangCauHoi[ThuTu].C !== MangCauHoi[ThuTu].KQ)
  {
    setShowC(false);
  }
  else
  if(MangCauHoi[ThuTu].D !== MangCauHoi[ThuTu].KQ)
  {
    setShowD(false);
  }
}
function FuncShowA()
{
  if(ShowA)
  {
    return  <div className='dapan1' onClick={FuncDapAn1}> A . {MangCauHoi[ThuTu].A}</div>
  }
  else{
    return  <div className='dapan1' onClick={FuncDapAn1}> A . </div>
  }
}
function FuncShowAMobie()
{
  if(ShowA)
  {
    return  <div className='dapan1Mobie' onClick={FuncDapAn1}> A . {MangCauHoi[ThuTu].A}</div>
  }
  else{
    return  <div className='dapan1Mobie' onClick={FuncDapAn1}> A . </div>
  }
}
function FuncShowB()
{
  if(ShowB)
  {
    return  <div className='dapan2' onClick={FuncDapAn2}> B . {MangCauHoi[ThuTu].B}</div>
  }
  else{
    return  <div className='dapan2' onClick={FuncDapAn2}> B . </div>
  }
}
function FuncShowBMobie()
{
  if(ShowB)
  {
    return  <div className='dapan2Mobie' onClick={FuncDapAn2}> B . {MangCauHoi[ThuTu].B}</div>
  }
  else{
    return  <div className='dapan2Mobie' onClick={FuncDapAn2}> B . </div>
  }
}
function FuncShowC()
{
  if(ShowC)
  {
    return  <div className='dapan3' onClick={FuncDapAn3}> C . {MangCauHoi[ThuTu].C}</div>
  }
  else{
    return  <div className='dapan3' onClick={FuncDapAn3}> C . </div>
  }
}
function FuncShowCMobie()
{
  if(ShowC)
  {
    return  <div className='dapan3Mobie' onClick={FuncDapAn3}> C . {MangCauHoi[ThuTu].C}</div>
  }
  else{
    return  <div className='dapan3Mobie' onClick={FuncDapAn3}> C . </div>
  }
}
function FuncShowD()
{
  if(ShowD)
  {
    return  <div className='dapan4' onClick={FuncDapAn4}> D . {MangCauHoi[ThuTu].D}</div>
  }
  else{
    return  <div className='dapan4' onClick={FuncDapAn4}> D . </div>
  }
}
function FuncShowDMobie()
{
  if(ShowD)
  {
    return  <div className='dapan4Mobie' onClick={FuncDapAn4}> D . {MangCauHoi[ThuTu].D}</div>
  }
  else{
    return  <div className='dapan4Mobie' onClick={FuncDapAn4}> D . </div>
  }
}
function FuncDapAn1()
{
  setShowKQ(true);
  if(MangCauHoi[ThuTu].A===MangCauHoi[ThuTu].KQ)
  {
    setSaiA(false);
    Dung.currentTime = 0;
    Dung.play();

    const db = getDatabase();
    set(ref(db,"KQ"+"/"),
           1
    )
    set(ref(db,"TRANG"+"/"),
           2
    )
  }
  else
  {
    setSaiA(true);
    Sai.currentTime = 0;
    Sai.play();

    const db = getDatabase();
    set(ref(db,"KQ"+"/"),
           0
    )
    set(ref(db,"TRANG"+"/"),
           2
    )
  }
}
function FuncDapAn2()
{
  setShowKQ(true);
  if(MangCauHoi[ThuTu].B===MangCauHoi[ThuTu].KQ)
  {
    setSaiB(false);
    Dung.currentTime = 0;
    Dung.play();

    const db = getDatabase();
    set(ref(db,"KQ"+"/"),
           1
    )
    set(ref(db,"TRANG"+"/"),
           2
    )
  }
  else
  {
    setSaiB(true);
    Sai.currentTime = 0;
    Sai.play();

    const db = getDatabase();
    set(ref(db,"KQ"+"/"),
           0
    )
    set(ref(db,"TRANG"+"/"),
           2
    )
  }
}
function FuncDapAn3()
{
  setShowKQ(true);
  if(MangCauHoi[ThuTu].C === MangCauHoi[ThuTu].KQ)
  {
    setSaiC(false);
    Dung.currentTime = 0;
    Dung.play();

    const db = getDatabase();
    set(ref(db,"KQ"+"/"),
           1
    )
    set(ref(db,"TRANG"+"/"),
           2
    )
  }
  else
  {
    setSaiC(true);
    Sai.currentTime = 0;
    Sai.play();

    const db = getDatabase();
    set(ref(db,"KQ"+"/"),
           0
    )
    set(ref(db,"TRANG"+"/"),
           2
    )
  }
}
function FuncDapAn4()
{
  setShowKQ(true);
  if(MangCauHoi[ThuTu].D===MangCauHoi[ThuTu].KQ)
  {
    setSaiD(false);
    Dung.currentTime = 0;
    Dung.play();

    const db = getDatabase();
    set(ref(db,"KQ"+"/"),
           1
    )
    set(ref(db,"TRANG"+"/"),
           2
    )
  }
  else
  {
    setSaiD(true);
    Sai.currentTime = 0;
    Sai.play();

    const db = getDatabase();
    set(ref(db,"KQ"+"/"),
           0
    )
    set(ref(db,"TRANG"+"/"),
           2
    )
  }
}
function TraLoiA()
{
  if((MangCauHoi[ThuTu].A === MangCauHoi[ThuTu].KQ) && ShowKQ ) return <div className='traloi1 showkq' onClick={FuncDapAn1}></div>
  else if(SaiA)
  {
    return <div className='traloi1 showsai' onClick={FuncDapAn1}></div>
  }
  else return <div className='traloi1' onClick={FuncDapAn1}></div>
}
function TraLoiAMobie()
{
  if((MangCauHoi[ThuTu].A === MangCauHoi[ThuTu].KQ) && ShowKQ ) return <div className='traloi1Mobie showkq' onClick={FuncDapAn1}></div>
  else if(SaiA)
  {
    return <div className='traloi1Mobie showsai' onClick={FuncDapAn1}></div>
  } else return <div className='traloi1Mobie' onClick={FuncDapAn1}></div>
}
function TraLoiB()
{
  if((MangCauHoi[ThuTu].B === MangCauHoi[ThuTu].KQ) && ShowKQ ) return <div className='traloi2 showkq' onClick={FuncDapAn2}></div>
  else if(SaiB)
  {
    return <div className='traloi2 showsai' onClick={FuncDapAn2}></div>
  }else return <div className='traloi2' onClick={FuncDapAn2}></div>
}
function TraLoiBMobie()
{
  if((MangCauHoi[ThuTu].B === MangCauHoi[ThuTu].KQ) && ShowKQ ) return <div className='traloi2Mobie showkq' onClick={FuncDapAn2}></div>
  else if(SaiB)
  {
    return <div className='traloi2Mobie showsai' onClick={FuncDapAn2}></div>
  }else return <div className='traloi2Mobie' onClick={FuncDapAn2}></div>
}
function TraLoiC()
{
  if((MangCauHoi[ThuTu].C === MangCauHoi[ThuTu].KQ) && ShowKQ ) return <div className='traloi3 showkq' onClick={FuncDapAn3}></div>
  else if(SaiC)
  {
    return <div className='traloi3 showsai' onClick={FuncDapAn3}></div>
  }else return <div className='traloi3' onClick={FuncDapAn3}></div>
}
function TraLoiCMobie()
{
  if((MangCauHoi[ThuTu].C === MangCauHoi[ThuTu].KQ) && ShowKQ ) return <div className='traloi3Mobie showkq' onClick={FuncDapAn3}></div>
  else if(SaiC)
  {
    return <div className='traloi3Mobie showsai' onClick={FuncDapAn3}></div>
  }else return <div className='traloi3Mobie' onClick={FuncDapAn3}></div>
}
function TraLoiD()
{
  if((MangCauHoi[ThuTu].D === MangCauHoi[ThuTu].KQ) && ShowKQ ) return <div className='traloi4 showkq' onClick={FuncDapAn4}></div>
  else if(SaiD)
  {
    return <div className='traloi4 showsai' onClick={FuncDapAn4}></div>
  }else return <div className='traloi4' onClick={FuncDapAn4}></div>
}
function TraLoiDMobie()
{
  if((MangCauHoi[ThuTu].D === MangCauHoi[ThuTu].KQ) && ShowKQ ) return <div className='traloi4Mobie showkq' onClick={FuncDapAn4}></div>
  else if(SaiD)
  {
    return <div className='traloi4Mobie showsai' onClick={FuncDapAn4}></div>
  }else return <div className='traloi4Mobie' onClick={FuncDapAn4}></div>
}
function HienThiContent()
{
  if(isDesktopOrLaptop)
  { 
    return(
      <div className='container'>
        <div className='row'>
            <div className='col-sm-3 phan1'></div>
            <div className='col-sm-6 phan2'>
                <div className='tocauhoi'>
                     <div className='socauhoi'>Câu hỏi {ThuTu}</div>
                     <div className='ndcauhoi'>{MangCauHoi[ThuTu].CauHoi}</div>
                </div>
            </div>
            <div className='col-sm-3 phan3'></div>
        </div>
        {/* hết 1 row */}
        <div className='row'>
            <div className='col-sm-2 phan1'></div>
            <div className='col-sm-8 phan2'>
                <div className='totraloi'>
                     {TraLoiA()}
                     {FuncShowA()}
                     {TraLoiB()}
                     {FuncShowB()}
                     {TraLoiC()}
                     {FuncShowC()}
                     {TraLoiD()}
                     {FuncShowD()}
                     <button type="button" onClick={Func5050} className="btn btn-info  nutnhan">Trợ Giúp 50/50</button>
                     <button type="button" onClick={FuncCauTiepTheo} className="btn btn-success  nutnhan1">Câu Tiếp Theo</button>
                </div>
            </div>
            <div className='col-sm-2 phan3'></div>
        </div>


      </div>
    )
  }
  else if(isTabletOrMobile)
  {
    return(
      <div className='container'>
      <div className='row'>
          <div className='col-1 phan1'></div>
          <div className='col-10 phan2'>
              <div className='tocauhoiMobie'>
                   <div className='socauhoiMobie'>Câu hỏi {ThuTu}</div>
                   <div className='ndcauhoiMobie'>{MangCauHoi[ThuTu].CauHoi}</div>
              </div>
          </div>
          <div className='col-1 phan3'></div>
      </div>
      {/* hết 1 row */}
      <div className='row'>
          <div className='col-0 phan1'></div>
          <div className='col-12 phan2'>
              <div className='totraloi'>
                   {TraLoiAMobie()}
                   {FuncShowAMobie()}
                   {TraLoiBMobie()}
                   {FuncShowBMobie()}
                   {TraLoiCMobie()}
                   {FuncShowCMobie()}
                   {TraLoiDMobie()}
                   {FuncShowDMobie()}
                   <button type="button" onClick={Func5050} className="btn btn-info  nutnhanMobie">Trợ Giúp 5050</button>
                   <button type="button" onClick={FuncCauTiepTheo} className="btn btn-success  nutnhan1Mobie">Câu Tiếp Theo</button>
              </div>
          </div>
          <div className='col-0 phan3'></div>
      </div>


    </div>
    ) 
  }
}
// end responsive 

// firebase




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