#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266WebServer.h>

#define PIN_LED 16

int statusCode, LanDau =0;;
String content;


bool TestWifi();
void LaunchWeb();
void SetupAP();
bool LongPress();
void WriteDataToFirebase();

ESP8266WebServer server(80);

void setup()
{
  Serial.begin(115200);
  Serial.println();
  Serial.println("Ngat ket noi wifi da ket noi truoc do");
  delay(10);
  pinMode(PIN_LED, OUTPUT); // GPIO16

  Serial.println();


  LaunchWeb();
  SetupAP();

}
///////////////////////////////////////////////////////////LOOP///////////////////////////////////////////////////////////
void loop() {
  server.handleClient();
}
void LaunchWeb()
{
  Serial.print("Local IP: ");
  Serial.println(WiFi.localIP());
  Serial.print("SoftAP IP: ");
  Serial.println(WiFi.softAPIP());
  CreateWebServer();
  // Start the server
  server.begin();
  Serial.println("Server started");
}

void SetupAP()
{
  WiFi.mode(WIFI_STA);
  Serial.println("");
  WiFi.softAP("Vòng Quay Tửu Lượng", "");
  LaunchWeb();
  Serial.println("over");
}

// 192.168.4.1
void CreateWebServer()
{
  server.on("/nut1", []()
  {
    if(LanDau ==0)
    {
      LanDau =-1;
    }
    else
    {
      String state = server.arg("TAT");
      if (state.length() > 0)
      {
        Serial.println(state);
      }
    }
    
    content ="";
    content += "<form action='nut2' method='post'>";
    content += "<input type='submit' value='ON' name='BAT'>";
    content += "</form>";

    server.send(200, "text/html", content);
  });

  server.on("/nut2", []()
  {
    String state = server.arg("BAT");
    if (state.length() > 0)
    {
      Serial.println(state);
    }
    content ="";
    content += "<form action='nut1' method='post'>";
    content += "<input type='submit' value='OFF' name='TAT'>";
    content += "</form>";


    server.send(200, "text/html", content);
    //      ESP.reset();
  });
}
