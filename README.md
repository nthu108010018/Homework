# Homework

/model_run/run to start gui mode
/angle_detect/run to start angle detection mode

change SSID ,Password in mbed_app.json
change ip address(char HOST) in main.cpp, close_gesture.py, close_angle.py

先將uLCD接上mbed上， compile

open new terminal, use sudo screen /dev/ttyACM0 to open screen
wait for wifi/mqtt connect
在screen 上打 /model_run/run 開啟GUI mode  (RPCFunction)
GUI mode以lab8 model_deploy改寫 其中ring :15度 slope:30度 parrallel:45度
在按下USER_BUTTON前 開啟 close_gesture.py 按下user_button後 mbed將publish 所選擇之threshold_angle到mqtt_broker, 
同時間python 將收到並且回傳指令關掉GUI mode

接下來在screen上打/angle_detect/run 準備開始angle_detection mode
同時打開close_angle.py
在led1閃爍之後開始讀取參考數據，接下來led3閃爍時需擺好角度，因為這代表之後將讀取三軸數據
接下來會將參考數據以及之後所取得數據做內積之後除絕對值求得cos再利用cos求得tan
最後利用atan()取得角度
若角度大於GUI mode選擇角度，將會published此角度到mqtt_broker 當pc/python端偵測角度超過5個之後就會回傳指令
用rpc::call關掉angle_detection mode
