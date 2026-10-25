#include "web_page.h"

// Create AsyncWebServer object on port 80
WebServer server(80);

void handleRoot(){
  server.send(200, "text/html", index_html); //Send web page
}

void webCtrlServer(){
  server.on("/", handleRoot);

  server.on("/version", [](){
    server.send(200, "text/plain", FW_VERSION);
  });

  server.on("/ota", [](){
    runOtaCheck();
    server.send(200, "text/plain", "OTA check started. Check Serial monitor.");
  });

  server.on("/js", [](){
    String jsonCmdWebString = server.arg(0);
    deserializeJson(jsonCmdReceive, jsonCmdWebString);
    jsonCmdReceiveHandler();
    serializeJson(jsonInfoHttp, jsonFeedbackWeb);
    server.send(200, "text/plane", jsonFeedbackWeb);
    jsonFeedbackWeb = "";
    jsonInfoHttp.clear();
    jsonCmdReceive.clear();
  });

  // Start server
  server.begin();
  Serial.println("Server Starts.");
}

void initHttpWebServer(){
  webCtrlServer();
}