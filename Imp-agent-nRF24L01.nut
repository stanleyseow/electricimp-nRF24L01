http.onrequest(function(req, resp){
    server.log("Got a HTTP request");
    if (req.method == "POST"){
        local body = http.urldecode(req.body)
        server.log(body.data);
        device.send("agentBuffers",body.data);
        //device.on(recvfromImp,impBuffer);
    } 
    resp.send(200, "<head><meta http-equiv=\"refresh\" content=\"1; url=http://stanleyseow.asuscomm.com/color/\"><html>OK</html></head>");

});
